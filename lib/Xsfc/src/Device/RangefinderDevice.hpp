#pragma once

#include "BusAwareDevice.hpp"
#include "BusDevice.hpp"

namespace Xsfc {

enum RangefinderDeviceType
{
  RANGEFINDER_DEFAULT = 0,
  RANGEFINDER_NONE = 1,
  RANGEFINDER_VL53L0X = 2,
  RANGEFINDER_MSP = 3,
  RANGEFINDER_MATEK3901 = 4,
  RANGEFINDER_MTF02 = 5,
  RANGEFINDER_MTF02P = 6,
  RANGEFINDER_MAX
};

constexpr int32_t RANGEFINDER_NO_NEW_DATA = -1;
constexpr int32_t RANGEFINDER_OUT_OF_RANGE = -2;
constexpr int32_t RANGEFINDER_HARDWARE_FAILURE = -3;

inline bool isMspRangefinder(int type)
{
  return type == RANGEFINDER_MSP || type == RANGEFINDER_MATEK3901 ||
         type == RANGEFINDER_MTF02 || type == RANGEFINDER_MTF02P;
}

inline int32_t rangefinderProfileMaxMm(int type)
{
  switch (type)
  {
    case RANGEFINDER_VL53L0X:
    case RANGEFINDER_MATEK3901: return 2000;
    case RANGEFINDER_MTF02: return 2500;
    case RANGEFINDER_MTF02P: return 6000;
    default: return 0; // Generic MSP: the user supplies the working-range limit.
  }
}

namespace Device {

class RangefinderDevice : public BusAwareDevice
{
public:
  typedef RangefinderDeviceType DeviceType;

  virtual int begin(BusDevice* bus) = 0;
  virtual int begin(BusDevice* bus, uint8_t addr) = 0;

  virtual DeviceType getType() const = 0;

  // Distance in mm, or a RANGEFINDER_* status (no data, out of range, I/O failure).
  virtual int32_t readRangeMm() = 0;

  // polling interval in microseconds
  virtual int getDelay() const = 0;

  // maximum reliable measuring distance in millimeters
  virtual int32_t getMaxRangeMm() const = 0;

  virtual bool testConnection() = 0;

  static const char** getNames();
  static const char* getName(DeviceType type);
};

} // namespace Device

} // namespace Xsfc
