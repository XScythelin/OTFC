#pragma once

#include "Device/RangefinderDevice.hpp"

namespace Otfc::Device::Rangefinder {

class RangefinderTFLuna : public RangefinderDevice
{
public:
  int begin(BusDevice* bus) final;
  int begin(BusDevice* bus, uint8_t addr) final;

  RangefinderDeviceType getType() const final;

  int32_t readRangeMm() final;
  int getDelay() const final;
  int32_t getMaxRangeMm() const final;

  bool testConnection() final;

private:
  bool readData(uint8_t* data, uint8_t size);
  bool readRawFrame(uint8_t* frame);
};

} // namespace Otfc::Device::Rangefinder
