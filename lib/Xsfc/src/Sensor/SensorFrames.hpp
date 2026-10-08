#pragma once

#include "Sensor/RangeMeasurement.hpp"
#include "Sensor/FlowMeasurement.hpp"

namespace Xsfc::Sensor {

struct BaroFrame
{
  bool healthy = false;
  int32_t altitudeBiasSamples = 1;
  float pressure = 0.f, altitudeGround = 0.f, vario = 0.f, sampleIntervalf = 0.f;
  uint32_t updateCount = 0, lastUpdateUs = 0;
};

inline bool baroFrameReady(const BaroFrame& frame, uint32_t now)
{
  return frame.healthy && frame.altitudeBiasSamples < 0 &&
      std::isfinite(frame.pressure) && std::isfinite(frame.altitudeGround) && std::isfinite(frame.vario) &&
      (uint32_t)(now - frame.lastUpdateUs) <= 200000;
}

struct RangeFeed
{
  int32_t raw = 0;
  uint8_t quality = 0;
  uint32_t timestamp = 0;
  uint32_t count = 0;
  bool seen = false;
};

struct FlowFeed
{
  int32_t motionX = 0, motionY = 0;
  uint8_t quality = 0;
  uint32_t timestamp = 0, interval = 0, count = 0;
  bool seen = false;
};

struct RangeFrame
{
  bool present = false, valid = false;
  uint8_t validSamples = 0, quality = 0;
  int32_t raw = 0, maxRangeMm = 0, rate = 0;
  float height = 0.f, distance = 0.f, vario = 0.f, sampleIntervalf = 0.f;
  uint32_t lastSampleUs = 0, updateCount = 0;
  RangeStatus status = RangeStatus::NO_DATA;
};

struct FlowFrame
{
  bool present = false, valid = false;
  uint8_t validSamples = 0;
  uint8_t quality = 0;
  int32_t motionX = 0, motionY = 0, rate = 0;
  float sampleIntervalf = 0.f, flowRateX = 0.f, flowRateY = 0.f, bodyRateX = 0.f, bodyRateY = 0.f;
  float velocityForward = 0.f, velocityRight = 0.f, velocityX = 0.f, velocityY = 0.f;
  uint32_t lastSampleUs = 0, processedCount = 0;
  FlowStatus status = FlowStatus::NO_DATA;
};

inline bool rangeFrameReady(const RangeFrame& frame, uint32_t now)
{
  return frame.present && frame.valid && frame.validSamples >= 3 &&
      (uint32_t)(now - frame.lastSampleUs) <= 200000 &&
      std::isfinite(frame.height) && frame.height > 0.f && std::isfinite(frame.vario);
}

inline bool flowFrameReady(const FlowFrame& frame, uint32_t now)
{
  return frame.present && frame.valid && frame.validSamples >= 3 &&
      (uint32_t)(now - frame.lastSampleUs) <= 200000 &&
      std::isfinite(frame.velocityX) && std::isfinite(frame.velocityY);
}

} // namespace Xsfc::Sensor
