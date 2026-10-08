#pragma once

#include "Utils/Filter.h"
#include <cmath>

namespace Xsfc::Sensor {

enum class RangeStatus : uint8_t { NO_DATA, VALID, OUT_OF_RANGE, LIMIT, TILT, IO_ERROR, STALE, BAD_INTERVAL };

inline const char* rangeStatusName(RangeStatus status)
{
  switch (status)
  {
    case RangeStatus::VALID: return "valid";
    case RangeStatus::OUT_OF_RANGE: return "out of range";
    case RangeStatus::LIMIT: return "configured/device range limit";
    case RangeStatus::TILT: return "excessive/invalid tilt";
    case RangeStatus::IO_ERROR: return "I/O failure";
    case RangeStatus::STALE: return "stale";
    case RangeStatus::BAD_INTERVAL: return "invalid sample interval";
    default: return "no data";
  }
}

class RangeMeasurement
{
public:
  static constexpr uint32_t TIMEOUT_US = 200000;

  void begin(const FilterConfig& filter)
  {
    _config = filter;
    _heightFilter.begin(filter, 50);
    _varioFilter.begin(FilterConfig(FILTER_PT1, 3), 50);
    valid = false;
    streak = 0;
    _initialized = false;
    _haveTime = false;
    status = RangeStatus::NO_DATA;
    height = distance = vario = sampleDt = 0.f;
  }

  bool sample(int32_t rawMm, float cosTilt, uint32_t now, int32_t maxMm)
  {
    const uint32_t elapsed = now - lastSampleUs;
    sampleDt = _haveTime && elapsed <= TIMEOUT_US ? elapsed * 1e-6f : 0.f;
    lastSampleUs = now;
    const bool repeatedTime = _haveTime && elapsed < 2000;
    _haveTime = true;
    if (rawMm <= 0) return invalidate(RangeStatus::OUT_OF_RANGE);
    if (maxMm > 0 && rawMm > maxMm) return invalidate(RangeStatus::LIMIT);
    if (!std::isfinite(cosTilt) || cosTilt < 0.9063078f || cosTilt > 1.0001f)
      return invalidate(RangeStatus::TILT);
    if (repeatedTime) return invalidate(RangeStatus::BAD_INTERVAL);

    const float rawHeight = rawMm * 0.001f * std::min(cosTilt, 1.f);
    if (!_initialized || sampleDt == 0.f)
    {
      _heightFilter.reset(rawHeight);
      _varioFilter.reset(0.f);
      height = rawHeight;
      vario = 0.f;
      streak = 0;
    }
    else
    {
      const int rate = std::max(1, (int)std::lround(1.f / sampleDt));
      _heightFilter.reconfigure(_config, rate);
      _varioFilter.reconfigure(FilterConfig(FILTER_PT1, 3), rate);
      const float previous = height;
      height = _heightFilter.update(rawHeight);
      vario = _varioFilter.update((height - previous) / sampleDt);
    }
    distance = height / cosTilt;
    _initialized = valid = true;
    if (streak < 3) ++streak;
    status = RangeStatus::VALID;
    return true;
  }

  bool invalidate(RangeStatus reason)
  {
    status = reason;
    _initialized = valid = false;
    streak = 0;
    sampleDt = vario = 0.f;
    return false;
  }

  bool fresh(uint32_t now) const
  {
    return _haveTime && (uint32_t)(now - lastSampleUs) <= TIMEOUT_US;
  }

  bool ready(uint32_t now) const { return valid && streak >= 3 && fresh(now); }

  bool valid = false;
  uint8_t streak = 0;
  uint32_t lastSampleUs = 0;
  RangeStatus status = RangeStatus::NO_DATA;
  float height = 0.f;
  float distance = 0.f;
  float vario = 0.f;
  float sampleDt = 0.f;

private:
  bool _initialized = false;
  bool _haveTime = false;
  FilterConfig _config;
  Utils::Filter _heightFilter;
  Utils::Filter _varioFilter;
};

} // namespace Xsfc::Sensor
