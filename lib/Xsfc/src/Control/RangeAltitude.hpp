#pragma once

#include "Control/VerticalMotion.hpp"

namespace Xsfc::Control {

class RangeAltitude
{
public:
  void update(bool armed, bool rangeReady, bool newSample, float height, float vario, float sampleDt,
      bool accelActive, float acceleration, float dt, float groundBias)
  {
    const bool predictionValid = dt > 0.f && dt <= 0.1f && std::isfinite(acceleration);
    if (!armed)
    {
      motion.height = height;
      motion.velocity = vario;
      motion.bias = groundBias;
      _initialized = false;
    }
    else
    {
      if (rangeReady && (!_initialized || !_wasReady || !predictionValid))
      {
        motion.height = height;
        motion.velocity = vario;
        _initialized = true;
      }
      if (_initialized && accelActive && predictionValid) motion.predict(acceleration, dt);
      if (rangeReady && newSample)
      {
        if (!accelActive || sampleDt <= 0.f)
        {
          motion.height = height;
          motion.velocity = vario;
        }
        else
        {
          const float error = height - motion.height;
          motion.height += error * std::clamp(3.5f * sampleDt, 0.f, 1.f);
          motion.velocity += error * 6.1f * sampleDt;
          motion.bias = std::clamp(motion.bias - error * 3.5f * 3.5f * 0.01f * sampleDt, -2.45f, 2.45f);
        }
      }
    }
    ready = rangeReady && std::isfinite(motion.height) && std::isfinite(motion.velocity);
    _wasReady = rangeReady;
  }

  VerticalMotion motion;
  bool ready = false;

private:
  bool _initialized = false;
  bool _wasReady = false;
};

} // namespace Xsfc::Control
