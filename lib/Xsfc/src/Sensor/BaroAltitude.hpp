#pragma once

#include "Utils/Math.hpp"

namespace Xsfc::Sensor {

class BaroAltitude
{
public:
  static constexpr uint32_t TIMEOUT_US = 200000;
  enum class Result { CALIBRATING, RESTARTED, READY };

  void reset()
  {
    *this = BaroAltitude{};
  }

  void resetWindow()
  {
    _count = 0;
    _mean = 0.0;
    _m2 = 0.0;
  }

  Result update(float pressure, uint32_t now, bool armed)
  {
    absolute = Utils::toAltitude(pressure);
    if (!calibrated)
    {
      if (_count > 0 && (uint32_t)(now - _lastPressureTime) > TIMEOUT_US) resetWindow();
      _lastPressureTime = now;
      height = vario = 0.0f;
      if (armed)
      {
        resetWindow();
        return Result::CALIBRATING;
      }
      if (_count == 0) _windowStart = now;
      ++_count;
      const double delta = pressure - _mean;
      _mean += delta / _count;
      _m2 += delta * (pressure - _mean);
      if ((uint32_t)(now - _windowStart) < 2000000 || _count < 2)
        return Result::CALIBRATING;

      const float tolerancePa = 101325.f * (1.f - std::pow(1.f - 1.5f / 44330.f, 5.254999f));
      if (std::sqrt(_m2 / (_count - 1)) > tolerancePa)
      {
        resetWindow();
        return Result::RESTARTED;
      }
      groundPressure = (float)_mean;
      groundAltitude = Utils::toAltitude(groundPressure);
      calibrated = true;
      _initialized = false;
    }

    const uint32_t elapsed = now - _lastUpdate;
    sampleDt = _initialized && elapsed > 0 && elapsed <= TIMEOUT_US ? elapsed * 1e-6f : 0.f;
    _lastUpdate = now;

    if (!armed)
    {
      groundPressure = pressure;
      groundAltitude = absolute;
      height = vario = 0.f;
    }
    else
    {
      const float relative = absolute - groundAltitude;
      if (!_initialized || sampleDt == 0.f)
      {
        height = relative;
        vario = 0.f;
      }
      else
      {
        const float previous = height;
        const float gain = sampleDt / (sampleDt + 1.f / (2.f * Utils::pi()));
        height += gain * (relative - height);
        vario = (height - previous) / sampleDt;
      }
    }
    _initialized = true;
    return Result::READY;
  }

  bool calibrated = false;
  float groundPressure = 101325.f;
  float groundAltitude = 0.f;
  float absolute = 0.f;
  float height = 0.f;
  float vario = 0.f;
  float sampleDt = 0.f;

private:
  bool _initialized = false;
  uint32_t _lastUpdate = 0;
  uint32_t _lastPressureTime = 0;
  uint32_t _windowStart = 0;
  uint32_t _count = 0;
  double _mean = 0.0;
  double _m2 = 0.0;
};

} // namespace Xsfc::Sensor
