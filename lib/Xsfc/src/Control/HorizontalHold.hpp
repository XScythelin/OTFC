#pragma once

#include "Control/Pid.h"

namespace Xsfc::Control {

class HorizontalHold
{
public:
  void begin(float positionGain, float maxSpeed, float maxAngle, float p, float i, float d, float f, int rate)
  {
    _positionGain = positionGain;
    _maxSpeed = maxSpeed;
    _maxAcceleration = 9.80665f * std::tan(maxAngle);
    for (auto* pid : {&_pidX, &_pidY})
    {
      pid->Kp = p;
      pid->Ki = i;
      pid->Kd = d;
      pid->Kf = f;
      pid->rate = rate;
      pid->iLimitLow = pid->oLimitLow = -_maxAcceleration;
      pid->iLimitHigh = pid->oLimitHigh = _maxAcceleration;
      pid->iReset = 0.f;
      pid->dtermFilter.begin(FilterConfig(FILTER_PT1, 10), rate);
      pid->ftermDerivative = false;
      pid->begin();
    }
    reset(0.f, 0.f);
  }

  void reset(float velocityX, float velocityY)
  {
    positionX = positionY = angleRoll = anglePitch = 0.f;
    _accelerationX = _accelerationY = 0.f;
    _pidX.resetIterm();
    _pidY.resetIterm();
    _pidX.dtermFilter.reset();
    _pidY.dtermFilter.reset();
    _pidX.prevMeasurement = velocityX;
    _pidY.prevMeasurement = velocityY;
    _pidX.outputSaturated = _pidY.outputSaturated = false;
    _haveSample = false;
  }

  void sample(float velocityX, float velocityY, float dt)
  {
    if (_haveSample && dt > 0.f && dt <= 0.2f)
    {
      positionX += velocityX * dt;
      positionY += velocityY * dt;
    }
    _haveSample = true;
  }

  void update(float velocityX, float velocityY, float yaw)
  {
    float desiredX = -_positionGain * positionX;
    float desiredY = -_positionGain * positionY;
    limitVector(desiredX, desiredY, _maxSpeed);
    _pidX.outputSaturated = saturated(_pidX, _accelerationX, desiredX - velocityX);
    _pidY.outputSaturated = saturated(_pidY, _accelerationY, desiredY - velocityY);
    _accelerationX = _pidX.update(desiredX, velocityX);
    _accelerationY = _pidY.update(desiredY, velocityY);
    limitVector(_accelerationX, _accelerationY, _maxAcceleration);
    const float forward = std::cos(yaw) * _accelerationX + std::sin(yaw) * _accelerationY;
    const float right = -std::sin(yaw) * _accelerationX + std::cos(yaw) * _accelerationY;
    anglePitch = std::atan2(forward, 9.80665f);
    // XSFC's quaternion rotates +Z thrust towards -Y for positive roll.
    angleRoll = -std::atan2(right, std::sqrt(9.80665f * 9.80665f + forward * forward));
  }

  float positionX = 0.f, positionY = 0.f;
  float angleRoll = 0.f, anglePitch = 0.f;

private:
  static void limitVector(float& x, float& y, float limit)
  {
    const float length = std::hypot(x, y);
    if (length > limit && length > 0.f)
    {
      x *= limit / length;
      y *= limit / length;
    }
  }

  static bool saturated(const Pid& pid, float actual, float error)
  {
    const float raw = pid.pTerm + pid.iTerm + pid.dTerm + pid.fTerm;
    return ((raw >= pid.oLimitHigh || raw > actual + 0.0001f) && error > 0.f) ||
           ((raw <= pid.oLimitLow || raw < actual - 0.0001f) && error < 0.f);
  }

  Pid _pidX, _pidY;
  float _positionGain = 0.f;
  float _maxSpeed = 0.f;
  float _maxAcceleration = 0.f;
  float _accelerationX = 0.f, _accelerationY = 0.f;
  bool _haveSample = false;
};

} // namespace Xsfc::Control
