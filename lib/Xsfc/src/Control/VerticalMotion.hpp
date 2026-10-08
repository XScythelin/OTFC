#pragma once

#include <algorithm>
#include <cmath>

namespace Xsfc::Control {

struct VerticalMotion
{
  void predict(float acceleration, float dt)
  {
    const float linearAcceleration = acceleration - bias;
    height += velocity * dt + 0.5f * linearAcceleration * dt * dt;
    velocity += linearAcceleration * dt;
  }

  void correct(float baroHeight, float baroVelocity, float dt, float positionGain, float velocityGain, float biasGain)
  {
    const float positionError = baroHeight - height;
    const float velocityError = baroVelocity - velocity;
    height += positionError * std::clamp(positionGain * dt, 0.f, 1.f);
    velocity += velocityError * std::clamp(velocityGain * dt, 0.f, 1.f);
    bias -= (positionError * positionGain * positionGain + velocityError * velocityGain * velocityGain) * biasGain * dt;
    bias = std::clamp(bias, -2.45f, 2.45f);
  }

  float height = 0.f;
  float velocity = 0.f;
  float bias = 0.f;
};

} // namespace Xsfc::Control
