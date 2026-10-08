#pragma once

#include "Utils/Filter.h"
#include "Utils/SensorAlignment.hpp"
#include <array>

namespace Xsfc::Sensor {

class FlowGyroWindow
{
public:
  void reset() { *this = FlowGyroWindow{}; }

  void add(float x, float y, uint32_t now)
  {
    if (!_started)
    {
      _lastUs = now;
      _started = true;
      return;
    }
    const uint32_t dt = now - _lastUs;
    _lastUs = now;
    if (dt == 0) return;
    if (dt > 20000 || !std::isfinite(x) || !std::isfinite(y))
    {
      reset();
      _lastUs = now;
      _started = true;
      return;
    }
    _binUs += dt;
    _binX += x * dt;
    _binY += y * dt;
    if (_binUs >= 4000)
    {
      _history[_next] = {now, _binUs, _binX / _binUs, _binY / _binUs};
      _next = (_next + 1) % _history.size();
      if (_count < _history.size()) ++_count;
      _binUs = 0;
      _binX = _binY = 0.f;
    }
  }

  bool average(uint32_t end, uint32_t interval, float& x, float& y) const
  {
    if (interval == 0 || interval > 200000) return false;
    float sumX = 0.f, sumY = 0.f;
    uint32_t covered = 0;
    const auto accumulate = [&](const Entry& entry)
    {
      const int32_t endOffset = (int32_t)(entry.end - end);
      const int32_t startOffset = endOffset - (int32_t)entry.duration;
      const int32_t overlap = std::min(endOffset, 0) - std::max(startOffset, -(int32_t)interval);
      if (overlap > 0)
      {
        covered += overlap;
        sumX += entry.x * overlap;
        sumY += entry.y * overlap;
      }
    };
    for (size_t i = 0; i < _count; ++i) accumulate(_history[i]);
    if (_binUs > 0) accumulate({_lastUs, _binUs, _binX / _binUs, _binY / _binUs});
    if (covered < interval * 0.95f) return false;
    x = sumX / covered;
    y = sumY / covered;
    return true;
  }

private:
  struct Entry { uint32_t end = 0; uint32_t duration = 0; float x = 0.f; float y = 0.f; };
  std::array<Entry, 128> _history{};
  size_t _next = 0, _count = 0;
  bool _started = false;
  uint32_t _lastUs = 0, _binUs = 0;
  float _binX = 0.f, _binY = 0.f;
};

enum class FlowStatus : uint8_t { NO_DATA, VALID, QUALITY, INTERVAL, RANGE, GYRO, STALE, SCALE, MOTION };

inline const char* flowStatusName(FlowStatus status)
{
  switch (status)
  {
    case FlowStatus::VALID: return "valid";
    case FlowStatus::QUALITY: return "low quality";
    case FlowStatus::INTERVAL: return "invalid integration interval";
    case FlowStatus::RANGE: return "rangefinder unavailable/too close";
    case FlowStatus::GYRO: return "gyro integration window unavailable";
    case FlowStatus::STALE: return "stale";
    case FlowStatus::SCALE: return "invalid scale/alignment";
    case FlowStatus::MOTION: return "invalid/excessive angular motion";
    default: return "no data";
  }
}

class FlowMeasurement
{
public:
  void begin(const FilterConfig& filter)
  {
    _config = filter;
    _filterX.begin(filter, 50);
    _filterY.begin(filter, 50);
    _qualityGood = false;
    _initialized = valid = false;
    streak = 0;
    status = FlowStatus::NO_DATA;
  }

  bool sample(float motionX, float motionY, uint8_t quality, uint32_t intervalUs, float radiansPerCount,
      uint8_t alignment, float gainX, float gainY, float gyroX, float gyroY, bool gyroValid,
      bool compensateGyro, bool rangeReady, float height, float minimumHeight, float yaw,
      float roll = 0.f, float pitch = 0.f, float verticalSpeed = 0.f)
  {
    if (quality >= 90) _qualityGood = true;
    else if (quality <= 25) _qualityGood = false;
    if (!_qualityGood) return invalidate(FlowStatus::QUALITY);
    if (intervalUs < 2000 || intervalUs > 200000) return invalidate(FlowStatus::INTERVAL);
    if (!rangeReady || !std::isfinite(height) || height < minimumHeight) return invalidate(FlowStatus::RANGE);
    if (!std::isfinite(radiansPerCount) || radiansPerCount <= 0.f || alignment > ALIGN_CW270_DEG_FLIP)
      return invalidate(FlowStatus::SCALE);
    if (compensateGyro && (!gyroValid || !std::isfinite(gyroX) || !std::isfinite(gyroY)))
      return invalidate(FlowStatus::GYRO);

    sampleDt = intervalUs * 1e-6f;
    float unusedZ = 0.f;
    alignSensorVector(motionX, motionY, unusedZ, alignment);
    rateX = motionX * radiansPerCount * gainX / sampleDt;
    rateY = motionY * radiansPerCount * gainY / sampleDt;
    bodyX = compensateGyro ? gyroX : 0.f;
    bodyY = compensateGyro ? gyroY : 0.f;
    if (!std::isfinite(rateX) || !std::isfinite(rateY) || !std::isfinite(yaw) ||
        !std::isfinite(roll) || !std::isfinite(pitch) || !std::isfinite(verticalSpeed) ||
        std::fabs(rateX) > 100.f || std::fabs(rateY) > 100.f)
      return invalidate(FlowStatus::MOTION);

    forward = -(rateY - bodyY) * height;
    right = (rateX - bodyX) * height;
    const float cy = std::cos(yaw), sy = std::sin(yaw);
    const float cr = std::cos(roll), sr = std::sin(roll);
    const float cp = std::cos(pitch), sp = std::sin(pitch);
    const float worldX = cy * cp * forward + (cy * sp * sr - sy * cr) * right +
        (cy * sp * cr + sy * sr) * verticalSpeed;
    const float worldY = sy * cp * forward + (sy * sp * sr + cy * cr) * right +
        (sy * sp * cr - cy * sr) * verticalSpeed;
    const int rate = std::max(1, (int)std::lround(1.f / sampleDt));
    _filterX.reconfigure(_config, rate);
    _filterY.reconfigure(_config, rate);
    if (!_initialized)
    {
      _filterX.reset(worldX);
      _filterY.reset(worldY);
    }
    velocityX = _filterX.update(worldX);
    velocityY = _filterY.update(worldY);
    _initialized = valid = true;
    if (streak < 3) ++streak;
    status = FlowStatus::VALID;
    return true;
  }

  bool invalidate(FlowStatus reason)
  {
    status = reason;
    valid = _initialized = false;
    streak = 0;
    return false;
  }

  bool valid = false;
  uint8_t streak = 0;
  FlowStatus status = FlowStatus::NO_DATA;
  float sampleDt = 0.f;
  float rateX = 0.f, rateY = 0.f, bodyX = 0.f, bodyY = 0.f;
  float forward = 0.f, right = 0.f, velocityX = 0.f, velocityY = 0.f;

private:
  bool _qualityGood = false;
  bool _initialized = false;
  FilterConfig _config;
  Utils::Filter _filterX, _filterY;
};

} // namespace Xsfc::Sensor
