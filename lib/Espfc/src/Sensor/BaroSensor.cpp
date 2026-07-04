#include "BaroSensor.hpp"
#include <functional>
#include <cmath>
#include <algorithm>

namespace Espfc::Sensor {

BaroSensor::BaroSensor(Model& model):
  _model(model),
  _baro(nullptr),
  _state(BARO_STATE_INIT),
  _wait(0),
  _counter(0),
  _pressureValid(false),
  _groundPressure(101325.0f),
  _calPressureMin(0.0f),
  _calPressureMax(0.0f),
  _calPressureSum(0.0f),
  _calPressureCount(0),
  _groundCalibrated(false)
{}

int BaroSensor::begin()
{
  if (!_model.baroActive() || !_model.state.baro.dev) return 0;

  _baro = _model.state.baro.dev;

  const int delay = _baro->getDelay(BARO_MODE_TEMP) + _baro->getDelay(BARO_MODE_PRESS);
  const int toGyroRate = (delay / _model.state.gyro.timer.interval) + 1; // number of gyro readings per cycle
  const int interval = _model.state.gyro.timer.interval * toGyroRate;
  const int rate = 1000000 / interval;
  const int biasSamples = 3 * rate;
  const auto internalFilter = FILTER_PT1;
  const auto internalCutoff = std::max((rate + 4) / 8, 1);

  _temperatureFilter.begin(FilterConfig(internalFilter, internalCutoff), rate);
  _pressureFilter.begin(FilterConfig(internalFilter, internalCutoff), rate);
  _altitudeFilter.begin(FilterConfig(internalFilter, internalCutoff), rate);
  _varioFilter.begin(FilterConfig(internalFilter, internalCutoff), rate);

  _temperatureMedianFilter.begin(FilterConfig(FILTER_MEDIAN3, 0), rate);
  _pressureMedianFilter.begin(FilterConfig(FILTER_MEDIAN3, 0), rate);

  _model.logger.info()
      .log(F("BARO INIT"))
      .log(FPSTR(Device::BaroDevice::getName(_baro->getType())))
      .log(rate)
      .logln(internalCutoff);

  _model.state.baro.rate = rate;
  _model.state.baro.altitudeBiasSamples = biasSamples;
  _model.state.baro.lastUpdateUs = 0;
  _groundPressure = _model.state.baro.pressure > 0.f ? _model.state.baro.pressure : 101325.0f;
  _calPressureMin = _groundPressure;
  _calPressureMax = _groundPressure;
  _calPressureSum = 0.0f;
  _calPressureCount = 0;
  _groundCalibrated = false;
  _baro->setMode(BARO_MODE_TEMP);

  return 1;
}

int BaroSensor::update()
{
  int status = read();

  return status;
}

int BaroSensor::read()
{
  if (!_baro || !_model.baroActive()) return 0;

  if ((int32_t)(micros() - _wait) < 0) return 0;

  Utils::Stats::Measure measure(_model.state.stats, COUNTER_BARO);

  // if(_model.config.debug.mode == DEBUG_BARO)
  // {
  //   _model.state.debug[0] = _state;
  // }

  switch (_state)
  {
    case BARO_STATE_INIT:
      _baro->setMode(BARO_MODE_TEMP);
      _state = BARO_STATE_TEMP_GET;
      _wait = micros() + _baro->getDelay(BARO_MODE_TEMP);
      return 0;
    case BARO_STATE_TEMP_GET:
      readTemperature();
      _baro->setMode(BARO_MODE_PRESS);
      _state = BARO_STATE_PRESS_GET;
      _wait = micros() + _baro->getDelay(BARO_MODE_PRESS);
      _counter = 1;
      return 1;
    case BARO_STATE_PRESS_GET:
      readPressure();
      if (_pressureValid)
      {
        updateAltitude();
      }
      if (--_counter > 0)
      {
        _baro->setMode(BARO_MODE_PRESS);
        _state = BARO_STATE_PRESS_GET;
        _wait = micros() + _baro->getDelay(BARO_MODE_PRESS);
      }
      else
      {
        _baro->setMode(BARO_MODE_TEMP);
        _state = BARO_STATE_TEMP_GET;
        _wait = micros() + _baro->getDelay(BARO_MODE_TEMP);
      }
      return 1;
      break;
    default: _state = BARO_STATE_INIT; break;
  }

  return 0;
}

void BaroSensor::readTemperature()
{
  float temp = _model.state.baro.temperatureRaw = _baro->readTemperature();
  // temp = _temperatureMedianFilter.update(temp);
  _model.state.baro.temperature = _temperatureFilter.update(temp);
}

void BaroSensor::readPressure()
{
  _pressureValid = false;
  float press = _model.state.baro.pressureRaw = _baro->readPressure();
  if (!std::isfinite(press) || press < 30000.0f || press > 120000.0f)
  {
    return;
  }
  // press = _pressureMedianFilter.update(press);
  _model.state.baro.pressure = _pressureFilter.update(press);
  _pressureValid = true;
}

void BaroSensor::updateAltitude()
{
  Espfc::BaroState& baro = _model.state.baro;

  baro.altitudeRaw = Utils::toAltitude(baro.pressure);
  baro.altitude = _altitudeFilter.update(baro.altitudeRaw);
  const float varioRaw = (baro.altitude - baro.altitudePrev) * baro.rate;

  updateGroundReference(baro.pressure, varioRaw, _model.isModeActive(MODE_ARMED));

  baro.altitudeGround = baro.altitude - baro.altitudeBias;
  baro.updateCount++;
  baro.lastUpdateUs = micros();

  baro.vario = _varioFilter.update(varioRaw);
  baro.altitudePrev = baro.altitude;

  if (_model.config.debug.mode == DEBUG_BARO)
  {
    _model.state.debug[0] = lrintf(baro.vario * 100.0f);     // cm/s
    _model.state.debug[1] = lrintf(baro.pressureRaw * 0.1f); // hPa x 10
    //_model.state.debug[1] = lrintf(baro.pressureRaw - 100000.0f); // Pa - 100000
    _model.state.debug[2] = lrintf(baro.temperatureRaw * 100.f); // deg C x 100
    _model.state.debug[3] = lrintf(baro.altitudeGround * 100.f); // cm
  }
}

void BaroSensor::updateGroundReference(float pressure, float vario, bool armed)
{
  Espfc::BaroState& baro = _model.state.baro;
  const int32_t rate = std::max<int32_t>(baro.rate, 1);
  const int32_t resetSamples = 3 * rate;

  if (baro.altitudeBiasSamples > 0)
  {
    if (baro.altitudeBiasSamples == resetSamples)
    {
      baro.altitudeBias = baro.altitude;
      _calPressureMin = pressure;
      _calPressureMax = pressure;
      _calPressureSum = 0.0f;
      _calPressureCount = 0;
    }

    baro.altitudeBiasSamples--;
    baro.altitudeBias += (baro.altitude - baro.altitudeBias) * (5.0f / rate);

    _calPressureMin = std::min(_calPressureMin, pressure);
    _calPressureMax = std::max(_calPressureMax, pressure);
    _calPressureSum += pressure;
    _calPressureCount++;

    // Reject unstable startup windows (propwash / handling / thermal settling)
    // and restart the initial ground-reference capture.
    const int32_t evalWindowSamples = std::max<int32_t>(rate / 2, 5);
    const float pressureSpan = _calPressureMax - _calPressureMin;
    if (_calPressureCount >= evalWindowSamples)
    {
      if (pressureSpan > 120.0f || std::fabs(vario) > 0.8f)
      {
        baro.altitudeBiasSamples = resetSamples;
      }
      _calPressureMin = pressure;
      _calPressureMax = pressure;
      _calPressureSum = 0.0f;
      _calPressureCount = 0;
    }

    _groundCalibrated = false;
    return;
  }

  if (baro.altitudeBiasSamples == 0)
  {
    _model.logger.info().log("BARO BIAS").logln(baro.altitudeBias);
    baro.altitudeBiasSamples--;
    _groundPressure = pressure;
    _groundCalibrated = true;
    _calPressureMin = pressure;
    _calPressureMax = pressure;
    _calPressureSum = 0.0f;
    _calPressureCount = 0;
    return;
  }

  // Keep slowly re-zeroing the ground level while disarmed to cancel
  // barometer self-heating drift.
  if (!armed && std::fabs(vario) < 0.5f)
  {
    baro.altitudeBias += (baro.altitude - baro.altitudeBias) * (1.0f / rate);
    _groundPressure += (pressure - _groundPressure) * (1.0f / rate);
  }
}

} // namespace Espfc::Sensor
