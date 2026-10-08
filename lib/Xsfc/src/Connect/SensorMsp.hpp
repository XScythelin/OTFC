#pragma once

#include "Connect/Msp.hpp"
#include <limits>

namespace Xsfc::Connect {

constexpr uint16_t MSP2_SENSOR_RANGEFINDER = 0x1F01;
constexpr uint16_t MSP2_SENSOR_OPTIC_FLOW = 0x1F02;

inline bool isSensorFeed(uint16_t cmd)
{
  return cmd == MSP2_SENSOR_RANGEFINDER || cmd == MSP2_SENSOR_OPTIC_FLOW;
}

inline int32_t sensorSigned32(uint32_t value)
{
  return value <= (uint32_t)std::numeric_limits<int32_t>::max() ? (int32_t)value :
      -1 - (int32_t)(std::numeric_limits<uint32_t>::max() - value);
}

struct RangePacket { uint8_t quality; int32_t distanceMm; };
struct FlowPacket { uint8_t quality; int32_t motionX; int32_t motionY; };

inline bool readRangePacket(MspMessage& message, RangePacket& packet)
{
  if (message.remain() != 5) return false;
  packet.quality = message.readU8();
  packet.distanceMm = sensorSigned32(message.readU32());
  return true;
}

inline bool readFlowPacket(MspMessage& message, FlowPacket& packet)
{
  if (message.remain() != 9) return false;
  packet.quality = message.readU8();
  packet.motionX = sensorSigned32(message.readU32());
  packet.motionY = sensorSigned32(message.readU32());
  return true;
}

} // namespace Xsfc::Connect
