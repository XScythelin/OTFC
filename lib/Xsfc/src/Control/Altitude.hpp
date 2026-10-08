#pragma once

#include "Model.h"
#include "Utils/Filter.h"
#include "VerticalMotion.hpp"
#include "RangeAltitude.hpp"
#include <cmath>

namespace Xsfc::Control {

class Altitude
{
public:
  Altitude(Model& model): _model(model) {}

  int begin()
  {
    _model.state.altitude.baroHeight = _model.state.baro.altitudeGround;
    _model.state.altitude.accelHeight = _model.state.baro.altitudeGround;
    _model.state.altitude.height = _model.state.baro.altitudeGround;
    _model.state.altitude.vario = _model.state.baro.vario;
    _model.state.altitude.target = _model.state.altitude.height;
    _model.state.altitude.accelVario = 0.0f;
    _model.state.altitude.accelBias = 0.0f;
    _model.state.altitude.baroUpdateCount = _model.state.baro.updateCount;
    _motion = {};
    _initialized = false;
    _clockStarted = false;
    _range = {};
    _groundBias = 0.f;
    _model.state.altitude.rangeUpdateCount = 0;
    _model.state.altitude.rangeHeight = _model.state.altitude.rangeVario = 0.f;
    _model.state.altitude.rangeReady = _model.state.altitude.rangeSource = _model.state.altitude.sensorLost = false;

    _accelFilter.begin(FilterConfig(FILTER_PT1, 20), _model.state.accel.timer.rate);

    return 1;
  }

  int update()
  {
    const uint32_t now = micros();
    const uint32_t elapsed = now - _lastUpdateUs;
    const float dt = _clockStarted ? elapsed * 1e-6f : _model.state.accel.timer.intervalf;
    _lastUpdateUs = now;
    _clockStarted = true;
    if (dt <= 0.f) return 0;
    const bool predictionValid = dt <= 0.1f;
    if (!predictionValid) _initialized = false;
    auto& altitude = _model.state.altitude;
    const auto& altCfg = _model.config.altHold;
    const auto baro = _model.baroSample();
    const bool armed = _model.isModeActive(MODE_ARMED);
    const bool ready = _model.baroActive() && Sensor::baroFrameReady(baro, now);
    // Do not retain an inertially drifted state across an unavailable barometer.
    // The first usable sample must reseed both position and velocity.
    if (!ready) _initialized = false;
    const bool accelActive = _model.accelActive();
    const float acceleration = accelActive ? _accelFilter.update(_model.state.accel.world.z - ACCEL_G) : 0.f;
    altitude.baroHeight = baro.altitudeGround;

    if (!armed)
    {
      if (accelActive && std::isfinite(acceleration) && std::fabs(acceleration) < 2.45f)
      {
        const float gain = std::clamp((float)altCfg.accelBiasAlphaGround * 0.05f * dt, 0.f, 1.f);
        _groundBias += (acceleration - _groundBias) * gain;
      }
      _motion.bias = _groundBias;
      _motion.height = _motion.velocity = 0.f;
      _initialized = false;
    }
    else
    {
      if (!_initialized && ready)
      {
        _motion.height = altitude.baroHeight;
        _motion.velocity = baro.vario;
        _initialized = true;
      }
      if (_initialized && predictionValid && accelActive && std::isfinite(acceleration))
        _motion.predict(acceleration, dt);

      if (ready && baro.updateCount != altitude.baroUpdateCount)
      {
        if (accelActive && baro.sampleIntervalf > 0.f)
        {
          const float sampleDt = baro.sampleIntervalf;
          const float positionGain = std::clamp((float)altCfg.baroPosWeight * 0.01f, 0.f, 10.f);
          const float velocityGain = std::clamp((float)altCfg.baroVarioWeight * 0.01f, 0.f, 10.f);
          const float biasGain = std::clamp((float)altCfg.accelBiasAlphaAir * 0.002f, 0.f, 0.1f);
          _motion.correct(altitude.baroHeight, baro.vario, sampleDt, positionGain, velocityGain, biasGain);
        }
        else
        {
          _motion.height = altitude.baroHeight;
          _motion.velocity = baro.vario;
        }
      }
    }
    altitude.baroUpdateCount = baro.updateCount;

    const auto rf = _model.rangeSample();

    _range.update(armed, Sensor::rangeFrameReady(rf, now), rf.updateCount != altitude.rangeUpdateCount,
        rf.height, rf.vario, rf.sampleIntervalf, accelActive, acceleration, dt, _groundBias);
    altitude.rangeUpdateCount = rf.updateCount;
    altitude.rangeHeight = _range.motion.height;
    altitude.rangeVario = _range.motion.velocity;
    altitude.rangeReady = _range.ready;

    altitude.accelHeight = altitude.height = _motion.height;
    altitude.accelVario = altitude.vario = _motion.velocity;
    altitude.accelBias = _motion.bias;

    if(_model.config.debug.mode == DEBUG_ALTITUDE)
    {
      _model.state.debug[0] = std::clamp(lrintf(altitude.baroHeight * 100.0f), -32000l, 32000l);               // baroAlt cm
      _model.state.debug[1] = std::clamp(lrintf(altitude.accelHeight * 100.0f), -32000l, 32000l);               // accelAlt cm
      _model.state.debug[2] = std::clamp(lrintf(altitude.height * 100.0f), -32000l, 32000l);                    // fusedAlt cm
      _model.state.debug[3] = std::clamp(lrintf(altitude.vario * 100.0f), -32000l, 32000l);                     // fusedVario cm/s
    }

    return 1;
  }

private:
  Model& _model;
  Utils::Filter _accelFilter;
  VerticalMotion _motion;
  bool _initialized = false;
  uint32_t _lastUpdateUs = 0;
  bool _clockStarted = false;
  float _groundBias = 0.f;
  RangeAltitude _range;
};

}
