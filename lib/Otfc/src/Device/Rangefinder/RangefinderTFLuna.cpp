#include "RangefinderTFLuna.hpp"
#include "Target/Target.h"

namespace {

constexpr uint8_t TFLUNA_I2C_ADDRESS = 0x10;
constexpr uint8_t TFLUNA_REG_DATA = 0x00;
constexpr uint8_t TFLUNA_DATA_SIZE = 6;
constexpr uint8_t TFLUNA_FRAME_SIZE = 9;

} // namespace

namespace Otfc::Device::Rangefinder {

bool RangefinderTFLuna::readData(uint8_t* data, uint8_t size)
{
  if (!_bus || !data || size == 0) return false;

  const int8_t read = _bus->read(_addr, TFLUNA_REG_DATA, size, data);
  return read == size;
}

bool RangefinderTFLuna::readRawFrame(uint8_t* frame)
{
#if defined(OTFC_I2C_0)
  if (!_bus || !frame) return false;
  if (_bus->getType() != BUS_I2C) return false;

  WireInstance.requestFrom((uint8_t)_addr, (uint8_t)TFLUNA_FRAME_SIZE);

  uint8_t read = 0;
  const uint32_t t0 = millis();
  while (read < TFLUNA_FRAME_SIZE && (millis() - t0) < 10)
  {
    if (!WireInstance.available()) continue;
    frame[read++] = (uint8_t)WireInstance.read();
  }
  if (read != TFLUNA_FRAME_SIZE) return false;

  if (frame[0] != 0x59 || frame[1] != 0x59) return false;

  uint8_t checksum = 0;
  for (uint8_t i = 0; i < TFLUNA_FRAME_SIZE - 1; i++) checksum += frame[i];
  return checksum == frame[TFLUNA_FRAME_SIZE - 1];
#else
  (void)frame;
  return false;
#endif
}

bool RangefinderTFLuna::testConnection()
{
  uint8_t data[TFLUNA_DATA_SIZE] = {0};
  if (readData(data, sizeof(data))) return true;

  uint8_t frame[TFLUNA_FRAME_SIZE] = {0};
  return readRawFrame(frame);
}

int RangefinderTFLuna::begin(BusDevice* bus)
{
  return begin(bus, TFLUNA_I2C_ADDRESS);
}

int RangefinderTFLuna::begin(BusDevice* bus, uint8_t addr)
{
  setBus(bus, addr);
  return testConnection() ? 1 : 0;
}

RangefinderDeviceType RangefinderTFLuna::getType() const
{
  return RANGEFINDER_TF_LUNA;
}

int32_t RangefinderTFLuna::readRangeMm()
{
  // Prefer full frame reads with checksum validation to reject corrupted samples.
  uint8_t frame[TFLUNA_FRAME_SIZE] = {0};
  if (readRawFrame(frame))
  {
    const uint16_t distanceCm = (uint16_t)frame[2] | ((uint16_t)frame[3] << 8);
    const uint16_t strength = (uint16_t)frame[4] | ((uint16_t)frame[5] << 8);

    if (distanceCm == 0 || strength == 0) return -1;

    return (int32_t)distanceCm * 10; // cm -> mm
  }

  // Fallback for targets where frame mode is not available/reliable.
  uint8_t data[TFLUNA_DATA_SIZE] = {0};
  if (!readData(data, sizeof(data))) return -1;

  const uint16_t distanceCm = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
  const uint16_t strength = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
  if (distanceCm == 0 || strength == 0) return -1;

  return (int32_t)distanceCm * 10; // cm -> mm
}

int RangefinderTFLuna::getDelay() const
{
  return 20000; // 50 Hz default output rate
}

int32_t RangefinderTFLuna::getMaxRangeMm() const
{
  return 8000; // up to 8 m nominal range
}

} // namespace Otfc::Device::Rangefinder
