#include "Device/BusDevice.hpp"
#include "Hal/Pgm.h"

namespace Otfc::Device {

const char** BusDevice::getNames()
{
  static const char* busDevChoices[] = {PSTR("NONE"), PSTR("AUTO"), PSTR("I2C"), PSTR("SPI"), PSTR("SLV"), NULL};
  return busDevChoices;
}

const char* BusDevice::getName(BusType type)
{
  if (type >= BUS_MAX) return PSTR("?");
  return getNames()[type];
}

} // namespace Otfc::Device
