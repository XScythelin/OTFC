#include "RangefinderDevice.hpp"
#include "Hal/Pgm.h"
#include <cstddef>

namespace Xsfc::Device {

const char** RangefinderDevice::getNames()
{
  static const char* devChoices[] = {
    PSTR("AUTO"), PSTR("NONE"), PSTR("VL53L0X"), PSTR("MSP"),
    PSTR("MATEK3901"), PSTR("MTF02"), PSTR("MTF02P"), NULL
  };
  return devChoices;
}

const char* RangefinderDevice::getName(DeviceType type)
{
  if (type >= RANGEFINDER_MAX) return PSTR("?");
  return getNames()[type];
}

} // namespace Xsfc::Device
