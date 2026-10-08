#pragma once

#include "Control/Pid.h"

namespace Xsfc::Control {

class AltHoldThrottle
{
public:
  void begin(Pid& pid, float hover, float minimum, float vario, bool groundStart,
      float previousThrottle = NAN)
  {
    pid.resetIterm();
    pid.ptermFilter.reset();
    pid.dtermFilter.reset();
    pid.ftermFilter.reset();
    pid.prevMeasurement = vario;
    pid.outputSaturated = false;
    _correction = groundStart ? minimum - hover :
        (std::isfinite(previousThrottle) ? std::clamp(previousThrottle, minimum, 1.f) - hover : 0.f);
    // Seed the slow trim where its configured bounds permit. The output filter
    // starts at the previous throttle even when the integral limit is smaller.
    if (!groundStart) pid.iTerm = std::clamp(_correction, pid.iLimitLow, pid.iLimitHigh);
    if (groundStart) pid.iTerm = _correction;
  }

  float update(Pid& pid, float setpoint, float vario, float hover, float minimum, float dt, bool preparingTakeoff)
  {
    if (preparingTakeoff) return minimum;
    pid.oLimitLow = minimum - hover;
    pid.oLimitHigh = 1.f - hover;
    const float previousOutput = pid.pTerm + pid.iTerm + pid.dTerm + pid.fTerm;
    const float error = setpoint - vario;
    pid.outputSaturated = (previousOutput >= pid.oLimitHigh && error > 0.f) ||
                         (previousOutput <= pid.oLimitLow && error < 0.f);
    const float correction = pid.update(setpoint, vario);
    const float gain = dt / (dt + 1.f / (2.f * Utils::pi() * 4.f));
    _correction += gain * (correction - _correction);
    return std::clamp(hover + _correction, minimum, 1.f);
  }

private:
  float _correction = 0.f;
};

} // namespace Xsfc::Control
