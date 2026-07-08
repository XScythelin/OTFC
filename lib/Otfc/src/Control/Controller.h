#pragma once

#include "Control/Altitude.hpp"
#include "Control/Rates.h"
#include "Model.h"

namespace Otfc::Control {

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
  bool _altHoldPrepareTakeoff;
  bool _altHoldStickCentered;
  uint8_t _sensorFaultMask;
  uint32_t _sensorFaultSinceUs;
};

} // namespace Otfc::Control
