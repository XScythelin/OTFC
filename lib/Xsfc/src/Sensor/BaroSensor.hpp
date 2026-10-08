#pragma once

#include "BaseSensor.h"
#include "Device/BaroDevice.hpp"
#include "Model.h"
#include "Utils/Filter.h"
#include "BaroAltitude.hpp"

namespace Xsfc::Sensor {

class BaroSensor : public BaseSensor
{
public:
  enum BaroState
  {
    BARO_STATE_INIT,
    BARO_STATE_TEMP_GET,
    BARO_STATE_PRESS_GET,
  };

  BaroSensor(Model& model);

  int begin();
  int update();
  int read();

private:
  void readTemperature();
  void readPressure();
  void updateAltitude();
  void publish();

  Model& _model;
  Device::BaroDevice* _baro;
  BaroState _state;
  Utils::Filter _temperatureFilter;
  uint32_t _wait;
  int32_t _counter;
  bool _pressureValid;
  bool _sampleError;
  BaroAltitude _altitude;
};

} // namespace Xsfc::Sensor
