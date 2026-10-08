#pragma once

#include "Control/Altitude.hpp"
#include "Control/Rates.h"
#include "Control/AltHoldStick.hpp"
#include "Control/AltHoldThrottle.hpp"
#include "Control/HoldSafety.hpp"
#include "Control/HorizontalHold.hpp"
#include "Model.h"

namespace Xsfc::Control {

class Controller
{
public:
  Controller(Model& model);
  int begin();
  int update();

  void outerLoopRobot();
  void innerLoopRobot();
  void outerLoop();
  void innerLoop();

  inline float getTpaFactor() const;
  inline void resetIterm();
  float calculateSetpointRate(int axis, float input) const;
  float calcualteAltHoldSetpoint() const;

private:
  void beginAltHold();
  void beginPosHold();
  void updatePosHold();
  void beginInnerLoop(size_t axis);
  void beginOuterLoop(size_t axis);

  Model& _model;
  Rates _rates;
  Utils::Filter _speedFilter;
  AltHoldStick _altHoldStick;
  float _altHoldVelocity = 0.f;
  AltHoldThrottle _altHoldThrottle;
  HoldSafety _holdSafety;
  HorizontalHold _horizontalHold;
};

} // namespace Xsfc::Control
