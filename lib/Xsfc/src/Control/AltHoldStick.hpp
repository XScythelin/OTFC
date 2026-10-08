#pragma once

#include <algorithm>
#include <cmath>

namespace Xsfc::Control {

struct AltHoldStick
{
  void begin(float height, float center, bool groundStart)
  {
    target = height;
    neutral = center;
    preparingTakeoff = groundStart;
    adjusting = false;
  }

  float climbRate(float stick, float deadband, float maxRate) const
  {
    const float delta = stick - neutral;
    if (std::fabs(delta) <= deadband) return 0.f;
    const float range = delta > 0.f ? 1.f - neutral - deadband : 1.f + neutral - deadband;
    const float command = std::copysign((std::fabs(delta) - deadband) / range, delta);
    return std::clamp(command, -1.f, 1.f) * maxRate;
  }

  float update(float stick, float height, float deadband, float maxRate)
  {
    const float rate = climbRate(stick, deadband, maxRate);
    if (preparingTakeoff)
    {
      target = height;
      if (rate <= 0.f && std::fabs(height) <= 0.5f) return 0.f;
      preparingTakeoff = false;
    }
    if (rate != 0.f || adjusting) target = height;
    adjusting = rate != 0.f;
    return rate;
  }

  static float limitVelocity(float desired, float previous, float dt)
  {
    return std::clamp(desired, previous - 9.80665f * 0.8f * dt, previous + 9.80665f * 0.5f * dt);
  }

  static float holdVelocity(float error, float kp, float maxRate, float dt)
  {
    const float acceleration = maxRate;
    float speed = 0.f;
    if (kp > 0.f)
    {
      const float linearDistance = acceleration / (kp * kp);
      speed = std::fabs(error) <= linearDistance ? kp * std::fabs(error) :
          std::sqrt(2.f * acceleration * (std::fabs(error) - 0.5f * linearDistance));
    }
    else
    {
      speed = std::sqrt(2.f * acceleration * std::fabs(error));
    }
    speed = std::min(speed, maxRate);
    if (dt > 0.f) speed = std::min(speed, std::fabs(error) / dt);
    return std::copysign(speed, error);
  }

  float neutral = 0.f;
  float target = 0.f;
  bool preparingTakeoff = false;
  bool adjusting = false;
};

} // namespace Xsfc::Control
