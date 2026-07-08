#include "RangefinderDevice.hpp"
#include "Hal/Pgm.h"
#include <cstddef>

namespace Otfc::Device {

const char** RangefinderDevice::getNames()
{
  static const char* devChoices[] = {PSTR("AUTO"), PSTR("NONE"), PSTR("VL53L0X"), PSTR("MSP"), PSTR("TF-LUNA"), NULL};
  return devChoices;
}

const char* RangefinderDevice::getName(DeviceType type)
{
  if (type >= RANGEFINDER_MAX) return PSTR("?");
  return getNames()[type];
}

} // namespace Otfc::Device
