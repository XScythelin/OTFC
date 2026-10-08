#pragma once

#include "Debug_Xsfc.h"
#include "Device/BaroDevice.hpp"

namespace Xsfc::Device::Baro {

class BaroMS5611 : public BaroDevice
{
public:
  int begin(BusDevice* bus) final;
  int begin(BusDevice* bus, uint8_t addr) final;

  BaroDeviceType getType() const final;

  float readTemperature() final;
  float readPressure() final;

  void setMode(BaroDeviceMode mode) final;
  int getDelay(BaroDeviceMode mode) const final;

  bool testConnection() final;

protected:
  bool     sendCommand(uint8_t cmd);
  uint32_t readADC();
  bool     readPROM();
  uint8_t  crc4(uint16_t prom[8]);

  uint16_t _c[6] = {};
  int64_t _dT = 0;
  int64_t _temperatureFirstOrder = 2000;
  bool _temperatureValid = false;
  bool _conversionStarted = false;
};

} // namespace Xsfc::Device::Baro
