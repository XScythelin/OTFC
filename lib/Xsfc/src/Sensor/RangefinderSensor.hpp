#pragma once

#include "BaseSensor.h"
#include "Device/RangefinderDevice.hpp"
#include "Model.h"
#include "Utils/Filter.h"
#include "RangeMeasurement.hpp"

namespace Xsfc::Sensor {

class RangefinderSensor : public BaseSensor
{
public:
  RangefinderSensor(Model& model);

  int begin();
  int update();
  int read();

private:
  int applySample(int32_t raw, uint32_t timestamp);
  void publish();

  Model& _model;
  Device::RangefinderDevice* _rangefinder;
  RangeMeasurement _measurement;
  uint32_t _wait;
  int32_t _maxRangeMm;
  bool _isMsp;
  uint32_t _lastExtCount;
  RangeStatus _lastStatus = RangeStatus::NO_DATA;
  bool _pollStarted = false;
  uint8_t _sampleQuality = 0;
};

} // namespace Xsfc::Sensor
