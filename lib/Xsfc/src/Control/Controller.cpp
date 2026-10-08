#include "Control/Controller.h"
#include "Utils/Math.hpp"
#include <algorithm>
#include <cmath>

namespace Xsfc::Control {

static float getAltHoldStickNeutral(const Model& model)
{
  const auto& altCfg = model.config.altHold;
  const float hoverCenter = Utils::map((float)altCfg.itermCenter, 0.f, 100.f, -1.f, 1.f);

  switch (altCfg.throttleMode)
  {
    case ALTHOLD_THROTTLE_MID:
      return 0.0f;
    case ALTHOLD_THROTTLE_HOVER:
      return hoverCenter;
    case ALTHOLD_THROTTLE_STICK:
    default:
      return model.isThrottleLow() ? 0.0f : model.state.input.ch[AXIS_THRUST];
  }
}

Controller::Controller(Model& model): _model(model), _rates{} {}

int Controller::begin()
{
  _rates.begin(_model.config.input);
  _speedFilter.begin(FilterConfig(FILTER_BIQUAD, 10), _model.state.loopTimer.rate);

  beginInnerLoop(AXIS_ROLL);
  beginInnerLoop(AXIS_PITCH);
  beginInnerLoop(AXIS_YAW);
  beginOuterLoop(AXIS_ROLL);
  beginOuterLoop(AXIS_PITCH);
  beginAltHold();
  beginPosHold();

  return 1;
}

int FAST_CODE_ATTR Controller::update()
{
  uint32_t startTime = 0;
  if (_model.config.debug.mode == DEBUG_PIDLOOP)
  {
    startTime = micros();
    _model.state.debug[0] = startTime - _model.state.loopTimer.last;
  }

  {
    Utils::Stats::Measure(_model.state.stats, COUNTER_OUTER_PID);
    resetIterm();
    switch (_model.config.mixer.type)
    {
      case FC_MIXER_GIMBAL: outerLoopRobot(); break;

      default: outerLoop(); break;
    }
  }

  {
    Utils::Stats::Measure(_model.state.stats, COUNTER_INNER_PID);
    switch (_model.config.mixer.type)
    {
      case FC_MIXER_GIMBAL: innerLoopRobot(); break;

      default: innerLoop(); break;
    }
  }

  if (_model.config.debug.mode == DEBUG_PIDLOOP)
  {
    _model.state.debug[2] = micros() - startTime;
  }

  return 1;
}

void Controller::outerLoopRobot()
{
  const float speedScale = 2.f;
  const float gyroScale = 0.1f;
  const float speed = _speedFilter.update(_model.state.output.ch[AXIS_PITCH] * speedScale +
                                          _model.state.gyro.adc[AXIS_PITCH] * gyroScale);
  float angle = 0;
  const auto& input = _model.state.input;
  const auto& levelConf = _model.config.level;

  if (true || _model.isModeActive(MODE_ANGLE))
  {
    angle = input.ch[AXIS_PITCH] * Utils::toRad(levelConf.angleLimit);
  }
  else
  {
    angle = _model.state.outerPid[AXIS_PITCH].update(input.ch[AXIS_PITCH], speed) * Utils::toRad(levelConf.rateLimit);
  }
  _model.state.setpoint.angle.set(AXIS_PITCH, angle);
  _model.state.setpoint.rate[AXIS_YAW] = input.ch[AXIS_YAW] * Utils::toRad(levelConf.rateLimit);

  if (_model.config.debug.mode == DEBUG_ANGLERATE)
  {
    _model.state.debug[0] = speed * 1000;
    _model.state.debug[1] = lrintf(Utils::toDeg(angle) * 10);
  }
}

void Controller::innerLoopRobot()
{
  // VectorFloat v(0.f, 0.f, 1.f);
  // v.rotate(_model.state.attitude.quaternion);
  // const float angle = acos(v.z);

  const auto& attitude = _model.state.attitude;
  const auto& setpoint = _model.state.setpoint;

  auto& output = _model.state.output;
  auto& innerPid = _model.state.innerPid;

  const float angle = std::max(abs(attitude.euler[AXIS_PITCH]), abs(attitude.euler[AXIS_ROLL]));
  const bool stabilize = angle < Utils::toRad(_model.config.level.angleLimit);
  if (stabilize)
  {
    output.ch[AXIS_PITCH] = innerPid[AXIS_PITCH].update(setpoint.angle[AXIS_PITCH], attitude.euler[AXIS_PITCH]);
    output.ch[AXIS_YAW] = innerPid[AXIS_YAW].update(setpoint.rate[AXIS_YAW], _model.state.gyro.adc[AXIS_YAW]);
  }
  else
  {
    resetIterm();
    output.ch[AXIS_PITCH] = 0.f;
    output.ch[AXIS_YAW] = 0.f;
  }

  if (_model.config.debug.mode == DEBUG_ANGLERATE)
  {
    _model.state.debug[2] = lrintf(Utils::toDeg(attitude.euler[AXIS_PITCH]) * 10);
    _model.state.debug[3] = lrintf(output.ch[AXIS_PITCH] * 1000);
  }
}

void FAST_CODE_ATTR Controller::outerLoop()
{
  // POSHOLD includes angle stabilization and range-based altitude hold.
  updatePosHold();

  // Roll/Pitch rates control
  if (_model.isAngleControlActive())
  {
    const float posHoldDeadband = _model.config.posHold.deadband * 0.01f;
    for (size_t i = 0; i < AXIS_COUNT_RP; i++)
    {
      float angleSetpoint;
      const bool centered = std::fabs(_model.state.input.ch[AXIS_ROLL]) <= posHoldDeadband &&
                            std::fabs(_model.state.input.ch[AXIS_PITCH]) <= posHoldDeadband;
      if (_model.state.posHold.engaged && centered)
      {
        // stick centered while holding: fly to the auto position-hold lean angle
        angleSetpoint = _model.state.posHold.angle[i];
      }
      else
      {
        angleSetpoint = Utils::toRad(_model.config.level.angleLimit) * _model.state.input.ch[i];
      }
      _model.state.setpoint.rate[i] = _model.state.outerPid[i].update(angleSetpoint, _model.state.attitude.euler[i]);
      // Preserve the summer angle-mode feedforward behavior (also for POSHOLD).
      _model.state.innerPid[i].fScale = 0.f;
    }
  }
  else
  {
    for (size_t i = 0; i < AXIS_COUNT_RP; i++)
    {
      _model.state.setpoint.rate[i] = calculateSetpointRate(i, _model.state.input.ch[i]);
    }
  }

  // Yaw rates control
  _model.state.setpoint.rate[AXIS_YAW] = calculateSetpointRate(AXIS_YAW, _model.state.input.ch[AXIS_YAW]);

  // thrust control: iNav-like. Alt-hold engages immediately and throttle stick
  // always commands vertical speed around the center deadband.
  const bool altHoldRequested = _model.isModeActive(MODE_ARMED) && _model.isAltHoldActive();
  auto& altitude = _model.state.altitude;
  const bool previousSource = altitude.rangeSource;
  const bool previousRangeLoss = _holdSafety.rangeLost;
  const bool rangeReady = _model.rangefinderReadyForHold() && altitude.rangeReady &&
      std::isfinite(altitude.rangeHeight) && std::isfinite(altitude.rangeVario);
  const bool baroReady = _model.baroReadyForAltHold() && std::isfinite(altitude.height) && std::isfinite(altitude.vario);
  const bool altHoldAllowed = _holdSafety.update(_model.isModeActive(MODE_ARMED), _model.state.mode.mask,
      _model.state.mode.maskSwitch, baroReady, rangeReady, _model.config.altHold.baroFallback,
      altitude.engaged && previousSource);
  altitude.rangeSource = _holdSafety.rangeSource;
  altitude.sensorLost = _holdSafety.rangeLost;
  if (!previousRangeLoss && altitude.sensorLost)
  {
    _model.logger.err().logln(F("RANGEHOLD sensor lost, manual throttle; switch mode off/on"));
    _model.state.buzzer.push(BUZZER_RX_SET);
  }
  const bool altHoldReady = altitude.rangeSource ? rangeReady : baroReady;
  const float height = _model.holdHeight();
  const float vario = _model.holdVario();

  if (altHoldAllowed)
  {
    if (HoldSafety::needsReset(altitude.engaged, previousSource, altitude.rangeSource))
    {
      _model.state.altitude.engaged = true;
      const float minThrust = -1.f; // Mixer maps -1 to the configured motor idle.
      _model.state.altitude.hoverThrottle = std::clamp(Utils::map((float)_model.config.altHold.itermCenter, 0.f, 100.f, -1.f, 1.f), minThrust, 1.f);

      _model.state.outerPid[AXIS_THRUST].resetIterm();
      _model.state.outerPid[AXIS_THRUST].ptermFilter.reset();
      _model.state.outerPid[AXIS_THRUST].dtermFilter.reset();
      _model.state.outerPid[AXIS_THRUST].prevMeasurement = height;
      _model.state.innerPid[AXIS_THRUST].resetIterm();
      const float deadband = std::clamp((float)_model.config.altHold.stickDeadband * 0.01f, 0.01f, 0.45f);
      const float neutral = std::clamp(getAltHoldStickNeutral(_model), -1.f + deadband + 0.02f, 1.f - deadband - 0.02f);
      const bool groundStart = _model.isThrottleLow() && std::fabs(height) <= 0.5f;
      _altHoldStick.begin(height, neutral, groundStart);
      _altHoldVelocity = groundStart ? 0.f : vario;
      _altHoldThrottle.begin(_model.state.innerPid[AXIS_THRUST], _model.state.altitude.hoverThrottle,
          minThrust, vario, groundStart, _model.state.output.ch[AXIS_THRUST]);
    }

    const bool wasAdjusting = _altHoldStick.adjusting;
    const float climbRate = _altHoldStick.update(_model.state.input.ch[AXIS_THRUST], height,
        std::clamp((float)_model.config.altHold.stickDeadband * 0.01f, 0.01f, 0.45f),
        std::clamp((float)_model.config.altHold.manualClimbRate, 10.f, 2000.f) * 0.01f);
    _model.state.altitude.target = _altHoldStick.target;
    if (wasAdjusting && !_altHoldStick.adjusting)
    {
      auto& positionPid = _model.state.outerPid[AXIS_THRUST];
      positionPid.resetIterm();
      positionPid.ptermFilter.reset();
      positionPid.dtermFilter.reset();
      positionPid.prevMeasurement = height;
    }
    if (_altHoldStick.preparingTakeoff)
    {
      _model.state.setpoint.rate[AXIS_THRUST] = 0.f;
      return;
    }

    float desiredVelocity = climbRate;
    const float maxRate = std::clamp((float)_model.config.altHold.manualClimbRate, 10.f, 2000.f) * 0.01f;
    if (climbRate == 0.f)
    {
      desiredVelocity = AltHoldStick::holdVelocity(_model.state.altitude.target - height,
          _model.state.outerPid[AXIS_THRUST].Kp, maxRate, _model.state.loopTimer.intervalf);
    }
    desiredVelocity = std::clamp(desiredVelocity, -maxRate, maxRate);
    _altHoldVelocity = AltHoldStick::limitVelocity(desiredVelocity, _altHoldVelocity, _model.state.loopTimer.intervalf);
    _model.state.setpoint.rate[AXIS_THRUST] = _altHoldVelocity;
  }
  else
  {
    _altHoldStick.preparingTakeoff = false;
    if (_model.state.altitude.engaged)
    {
      if (altHoldRequested && !altHoldReady && !altitude.rangeSource)
        _model.logger.err().logln(F("ALTHOLD barometer unavailable, manual throttle"));
      _model.state.outerPid[AXIS_THRUST].resetIterm();
      _model.state.innerPid[AXIS_THRUST].resetIterm();
    }
    _model.state.altitude.target = height;
    _model.state.altitude.engaged = false;
    _model.state.setpoint.rate[AXIS_THRUST] = _model.state.input.ch[AXIS_THRUST];
  }

  // debug
  if (_model.config.debug.mode == DEBUG_ANGLERATE)
  {
    for (size_t i = 0; i < AXIS_COUNT_RPY; ++i)
    {
      _model.state.debug[i] = lrintf(Utils::toDeg(_model.state.setpoint.rate[i]));
    }
  }
}

void FAST_CODE_ATTR Controller::innerLoop()
{
  // Roll/Pitch/Yaw rates control
  const float tpaFactor = getTpaFactor();
  const auto& setpoint = _model.state.setpoint;

  auto& innerPid = _model.state.innerPid;
  auto& output = _model.state.output;

  for (size_t i = 0; i < AXIS_COUNT_RPY; ++i)
  {
    output.ch[i] = innerPid[i].update(setpoint.rate[i], _model.state.gyro.adc[i]) * tpaFactor;
  }

  // thrust control: before engage -> manual stick (sits on the ground);
  // after engage -> hover baseline + vel PID climb-rate trim
  if (_model.isAltHoldActive() && _model.state.altitude.engaged)
  {
    const float minThrust = -1.f;
    const float hoverThrottle = std::clamp(_model.state.altitude.hoverThrottle, minThrust, 1.f);
    output.ch[AXIS_THRUST] = _altHoldThrottle.update(innerPid[AXIS_THRUST], setpoint.rate[AXIS_THRUST],
        _model.holdVario(), hoverThrottle, minThrust, _model.state.loopTimer.intervalf,
        _altHoldStick.preparingTakeoff);
  }
  else
  {
    innerPid[AXIS_THRUST].resetIterm();
    innerPid[AXIS_THRUST].oLimitLow = -0.5f;
    innerPid[AXIS_THRUST].oLimitHigh = 0.5f;
    output.ch[AXIS_THRUST] = _model.state.input.ch[AXIS_THRUST];
  }

  if (_model.config.debug.mode == DEBUG_STACK)
  {
    _model.state.debug[0] = std::clamp(lrintf(setpoint.rate[AXIS_THRUST] * 1000.0f), -3000l, 3000l);    // hi mem
    _model.state.debug[1] = std::clamp(lrintf(_model.holdVario() * 1000.0f), -30000l, 30000l); // lo mem
    _model.state.debug[2] = std::clamp(lrintf(_model.holdHeight() * 100.0f), -30000l, 30000l); // curr
    _model.state.debug[3] = std::clamp(lrintf(innerPid[AXIS_THRUST].error * 1000.0f), -30000l, 30000l); // p
    _model.state.debug[4] = std::clamp(lrintf(innerPid[AXIS_THRUST].pTerm * 1000.0f), -3000l, 3000l);
    _model.state.debug[5] = std::clamp(lrintf(innerPid[AXIS_THRUST].iTerm * 1000.0f), -3000l, 3000l);
    _model.state.debug[6] = std::clamp(lrintf(innerPid[AXIS_THRUST].dTerm * 1000.0f), -3000l, 3000l);
    _model.state.debug[7] = std::clamp(lrintf(innerPid[AXIS_THRUST].fTerm * 1000.0f), -3000l, 3000l);
  }

  // debug
  if (_model.config.debug.mode == DEBUG_ITERM_RELAX)
  {
    _model.state.debug[0] = lrintf(Utils::toDeg(innerPid[AXIS_ROLL].itermRelaxBase));
    _model.state.debug[1] = lrintf(innerPid[AXIS_ROLL].itermRelaxFactor * 100.0f);
    _model.state.debug[2] = lrintf(Utils::toDeg(innerPid[AXIS_ROLL].iTermError));
    _model.state.debug[3] = lrintf(innerPid[AXIS_ROLL].iTerm * 1000.0f);
  }
}

float Controller::calcualteAltHoldSetpoint() const
{
  float thrust = _model.state.input.ch[AXIS_THRUST]; // -1..1, 0 = center

  const float deadband = std::clamp((float)_model.config.altHold.stickDeadband * 0.01f, 0.01f, 0.45f);
  thrust = Utils::deadband(thrust, deadband);
  if (thrust == 0.f) return 0.f;

  const float span = 1.0f - deadband;
  const float climbRateNorm = Utils::map3(thrust, -span, 0.f, span, -1.0f, 0.f, 1.0f);

  // Symmetric manual climb command around neutral, scaled by configured max climb rate.
  const float maxManualClimbRate = std::clamp((float)_model.config.altHold.manualClimbRate, 10.f, 2000.f) * 0.01f; // cm/s -> m/s
  return climbRateNorm * maxManualClimbRate;
}

float Controller::getTpaFactor() const
{
  if (_model.config.controller.tpaScale == 0) return 1.f;
  float t = Utils::clamp(_model.state.input.us[AXIS_THRUST], (float)_model.config.controller.tpaBreakpoint, 2000.f);
  return Utils::map(t, (float)_model.config.controller.tpaBreakpoint, 2000.f, 1.f,
                    1.f - ((float)_model.config.controller.tpaScale * 0.01f));
}

void Controller::resetIterm()
{
  if (!_model.isModeActive(MODE_ARMED) // when not armed
      || (!_model.isAirModeActive() && _model.config.iterm.lowThrottleZeroIterm &&
          _model.isThrottleLow() &&
          !(_model.isAltHoldActive() && _model.state.altitude.engaged))
  )
  {
    for (size_t i = 0; i < AXIS_COUNT_RPY; i++)
    {
      _model.state.innerPid[i].resetIterm();
      _model.state.outerPid[i].resetIterm();
    }
  }
  if (!_model.isModeActive(MODE_ARMED))
  {
    //_model.state.innerPid[AXIS_THRUST].resetIterm();
  }
}

float Controller::calculateSetpointRate(int axis, float input) const
{
  if (axis == AXIS_YAW) input *= -1.f;
  return _rates.getSetpoint(axis, input);
}

void Controller::beginInnerLoop(size_t axis)
{
  const int pidFilterRate = _model.state.loopTimer.rate;
  float pidScale[] = {1.f, 1.f, 1.f};
  if (_model.config.mixer.type == FC_MIXER_GIMBAL)
  {
    pidScale[AXIS_YAW] = 0.2f;   // ROBOT
    pidScale[AXIS_PITCH] = 20.f; // ROBOT
  }

  const auto& pc = _model.config.pid[axis];
  const auto& dtermConf = _model.config.dterm;

  auto& pid = _model.state.innerPid[axis];
  pid.Kp = (float)pc.P * PTERM_SCALE * pidScale[axis];
  pid.Ki = (float)pc.I * ITERM_SCALE * pidScale[axis];
  pid.Kd = (float)pc.D * DTERM_SCALE * pidScale[axis];
  pid.Kf = (float)pc.F * FTERM_SCALE * pidScale[axis];
  pid.iLimitLow = -_model.config.iterm.limit * 0.01f;
  pid.iLimitHigh = _model.config.iterm.limit * 0.01f;
  pid.oLimitLow = -0.66f;
  pid.oLimitHigh = 0.66f;
  pid.rate = pidFilterRate;
  pid.dtermNotchFilter.begin(dtermConf.notchFilter, pidFilterRate);
  if (dtermConf.dynLpfFilter.cutoff > 0)
  {
    pid.dtermFilter.begin(FilterConfig((FilterType)dtermConf.filter.type, dtermConf.dynLpfFilter.cutoff),
                          pidFilterRate);
  }
  else
  {
    pid.dtermFilter.begin(dtermConf.filter, pidFilterRate);
  }
  pid.dtermFilter2.begin(dtermConf.filter2, pidFilterRate);
  pid.ftermFilter.begin(_model.config.input.filterDerivative, pidFilterRate);
  pid.itermRelaxFilter.begin(FilterConfig(FILTER_PT1, _model.config.iterm.relaxCutoff), pidFilterRate);
  if (axis == AXIS_YAW)
  {
    pid.itermRelax = (_model.config.iterm.relax == ITERM_RELAX_RPY || _model.config.iterm.relax == ITERM_RELAX_RPY_INC)
                         ? _model.config.iterm.relax
                         : ITERM_RELAX_OFF;
    pid.ptermFilter.begin(_model.config.yaw.filter, pidFilterRate);
  }
  else
  {
    pid.itermRelax = _model.config.iterm.relax;
  }
  pid.begin();
}

void Controller::beginOuterLoop(size_t axis)
{
  const int pidFilterRate = _model.state.loopTimer.rate;
  const auto& pc = _model.config.pid[FC_PID_LEVEL];

  auto& pid = _model.state.outerPid[axis];
  pid.Kp = (float)pc.P * LEVEL_PTERM_SCALE;
  pid.Ki = (float)pc.I * LEVEL_ITERM_SCALE;
  pid.Kd = (float)pc.D * LEVEL_DTERM_SCALE;
  pid.Kf = (float)pc.F * LEVEL_FTERM_SCALE;
  pid.iLimitHigh = Utils::toRad(_model.config.level.rateLimit * 0.1f);
  pid.iLimitLow = -pid.iLimitHigh;
  pid.oLimitHigh = Utils::toRad(_model.config.level.rateLimit);
  pid.oLimitLow = -pid.oLimitHigh;
  pid.rate = pidFilterRate;
  pid.ptermFilter.begin(_model.config.level.ptermFilter, pidFilterRate);
  // pid.iLimit = 0.3f; // ROBOT
  // pid.oLimit = 1.f;  // ROBOT
  pid.begin();
}

void Controller::beginAltHold()
{
  const auto& pcAlt = _model.config.pid[FC_PID_ALT];

  auto& posPid = _model.state.outerPid[AXIS_THRUST];
  posPid.Kp = (float)pcAlt.P * 0.01f;
  posPid.Ki = posPid.Kd = posPid.Kf = 0.f;
  posPid.iLimitLow = -1.0f;
  posPid.iLimitHigh = 1.0f;
  posPid.iReset = 0.0f;
  posPid.oLimitLow = -2.0f;
  posPid.oLimitHigh = 4.0f;
  posPid.rate = _model.state.loopTimer.rate;
  posPid.ptermFilter.begin(FilterConfig(FILTER_PT1, 5), _model.state.loopTimer.rate);
  posPid.dtermFilter.begin(FilterConfig(FILTER_PT1, 5), _model.state.loopTimer.rate);
  posPid.ftermDerivative = false;
  posPid.begin();

  const auto& pc = _model.config.pid[FC_PID_VEL];

  // vel pid produces a throttle correction around hover baseline (range roughly -0.6..0.6)
  auto& pid = _model.state.innerPid[AXIS_THRUST];
  const float throttleSpan = _model.state.mixer.maxThrottle - _model.state.mixer.minThrottle;
  const float pwmScale = 1000.f / std::max(throttleSpan, 1.f);
  // Navigation gains only; leave summer rate-PID scales and algorithm intact.
  pid.Kp = (float)pc.P * (0.2f / 66.7f) * pwmScale;
  pid.Ki = (float)pc.I * 0.01f * pwmScale;
  pid.Kd = (float)pc.D * 0.002f * pwmScale;
  pid.Kf = (float)pc.F * VEL_FTERM_SCALE;
  const float integralLimit = std::clamp((float)_model.config.altHold.itermRange * 0.02f, 0.f, 1.f);
  pid.iLimitLow = -integralLimit;
  pid.iLimitHigh = integralLimit;
  pid.iReset = 0.0f;
  pid.oLimitLow = -0.5f;
  pid.oLimitHigh = 0.5f;
  pid.rate = _model.state.loopTimer.rate;
  pid.ptermFilter.begin(FilterConfig(FILTER_PT1, 5), _model.state.loopTimer.rate);
  pid.dtermFilter.begin(FilterConfig(FILTER_PT1, 5), _model.state.loopTimer.rate);
  pid.ftermDerivative = false;
  pid.begin();
}

void Controller::beginPosHold()
{
  const auto& pcPos = _model.config.pid[FC_PID_POS];
  const auto& pcVel = _model.config.pid[FC_PID_POSR];
  _horizontalHold.begin(pcPos.P * 0.01f, _model.config.navigation.maxHorizontalSpeed * 0.01f,
      Utils::toRad((float)_model.config.posHold.angleLimit), pcVel.P / 20.f, pcVel.I / 100.f,
      pcVel.D / 100.f, pcVel.F / 100.f, _model.state.loopTimer.rate);
  _model.state.posHold = {};
}

void FAST_CODE_ATTR Controller::updatePosHold()
{
  auto& ph = _model.state.posHold;
  const auto& cfg = _model.config.posHold;

  const auto flow = _model.flowSample();
  const bool requested = _model.state.mode.maskSwitch & POSITION_HOLD_MODES;
  const bool armed = _model.isModeActive(MODE_ARMED);
  if (!requested || !armed)
  {
    ph.sensorLost = false;
    ph.monitoring = false;
  }
  const bool ready = _model.flowReadyForHold() && _model.rangefinderReadyForHold() &&
      _model.state.altitude.rangeReady && !_model.state.altitude.sensorLost &&
      std::isfinite(_model.state.altitude.rangeHeight) && std::isfinite(_model.state.altitude.rangeVario);
  if (ph.monitoring && !ready && !ph.sensorLost)
  {
    ph.sensorLost = true;
    _model.logger.err().logln(F("POSITION HOLD sensor lost, manual attitude; switch mode off/on"));
    _model.state.buzzer.push(BUZZER_RX_SET);
  }
  if (_model.isPosHoldActive() && armed && ready && !ph.sensorLost) ph.monitoring = true;
  const bool groundPrepared = _model.isModeActive(MODE_POSHOLD) &&
      (!_model.state.altitude.engaged || _altHoldStick.preparingTakeoff);
  const bool active = _model.isPosHoldActive() && _model.isAngleControlActive() && armed &&
      ready && !ph.sensorLost && !groundPrepared;

  if (!active)
  {
    ph.engaged = false;
    ph.posX = ph.posY = 0.0f;
    ph.angle[0] = ph.angle[1] = 0.0f;
    _horizontalHold.reset(flow.velocityX, flow.velocityY);
    ph.lastFlowCount = flow.processedCount;
    return;
  }

  const float deadband = cfg.deadband * 0.01f;
  const bool centered = std::fabs(_model.state.input.ch[AXIS_ROLL]) <= deadband &&
                        std::fabs(_model.state.input.ch[AXIS_PITCH]) <= deadband;
  ph.velX = flow.velocityX;
  ph.velY = flow.velocityY;
  if (!centered)
  {
    _horizontalHold.reset(ph.velX, ph.velY);
    ph.engaged = false;
    ph.posX = ph.posY = ph.angle[AXIS_ROLL] = ph.angle[AXIS_PITCH] = 0.f;
    ph.lastFlowCount = flow.processedCount;
    return;
  }
  if (!ph.engaged) _horizontalHold.reset(ph.velX, ph.velY);
  if (flow.processedCount != ph.lastFlowCount)
  {
    _horizontalHold.sample(ph.velX, ph.velY, flow.sampleIntervalf);
    ph.lastFlowCount = flow.processedCount;
  }
  _horizontalHold.update(ph.velX, ph.velY, _model.state.attitude.euler.z);
  ph.engaged = true;
  ph.posX = _horizontalHold.positionX;
  ph.posY = _horizontalHold.positionY;
  ph.angle[AXIS_ROLL] = _horizontalHold.angleRoll;
  ph.angle[AXIS_PITCH] = _horizontalHold.anglePitch;
}

} // namespace Xsfc::Control
