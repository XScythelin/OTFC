#include "BaroSensor.hpp"
#include <functional>
#include <cmath>
#include <algorithm>

namespace Xsfc::Sensor {

BaroSensor::BaroSensor(Model& model):
  _model(model),
  _baro(nullptr),
  _state(BARO_STATE_INIT),
  _wait(0),
  _counter(0),
  _pressureValid(false),
  _sampleError(false)
{}

int BaroSensor::begin()
{
  if (!_model.baroActive() || !_model.state.baro.dev) return 0;

  _baro = _model.state.baro.dev;

  const int delay = _baro->getDelay(BARO_MODE_TEMP) + _baro->getDelay(BARO_MODE_PRESS);
  const int toGyroRate = (delay / _model.state.gyro.timer.interval) + 1; // number of gyro readings per cycle
  const int interval = _model.state.gyro.timer.interval * toGyroRate;
  const int rate = 1000000 / interval;
  const auto internalFilter = FILTER_PT1;
  const auto internalCutoff = std::max((rate + 4) / 8, 1);

  _temperatureFilter.begin(FilterConfig(internalFilter, internalCutoff), rate);

  _model.logger.info()
      .log(F("BARO INIT"))
      .log(FPSTR(Device::BaroDevice::getName(_baro->getType())))
      .log(rate)
      .logln(internalCutoff);

  _model.state.baro.rate = rate;
  _model.state.baro.altitudeBiasSamples = 1;
  _model.state.baro.lastUpdateUs = 0;
  _model.state.baro.healthy = false;
  _model.state.baro.sampleIntervalf = 0.f;
  _model.state.baro.updateCount = 0;
  _altitude.reset();
  _state = BARO_STATE_INIT;
  _wait = 0;
  _sampleError = false;
  publish();

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
  if (std::isfinite(temp))
  {
    if (_model.state.baro.updateCount == 0) _temperatureFilter.reset(temp);
    _model.state.baro.temperature = _temperatureFilter.update(temp);
  }
  else
  {
    _model.state.baro.healthy = false;
    if (!_sampleError) _model.logger.err().logln(F("BARO invalid temperature"));
    _sampleError = true;
    publish();
  }
}

void BaroSensor::readPressure()
{
  _pressureValid = false;
  float press = _model.state.baro.pressureRaw = _baro->readPressure();
  if (!std::isfinite(_model.state.baro.temperatureRaw) || !std::isfinite(press) || press < 30000.0f || press > 120000.0f)
  {
    _model.state.baro.healthy = false;
    if (!_sampleError) _model.logger.err().logln(F("BARO invalid sample"));
    _sampleError = true;
    if (!_altitude.calibrated) _altitude.resetWindow();
    publish();
    return;
  }
  if (_sampleError) _model.logger.info().logln(F("BARO samples recovered"));
  _sampleError = false;
  _model.state.baro.pressure = press;
  _pressureValid = true;
}

void BaroSensor::updateAltitude()
{
  Xsfc::BaroState& baro = _model.state.baro;

  const uint32_t now = micros();
  const bool wasCalibrated = _altitude.calibrated;
  const auto result = _altitude.update(baro.pressure, now, _model.isModeActive(MODE_ARMED));
  if (result == BaroAltitude::Result::RESTARTED)
    _model.logger.info().logln(F("BARO calibration unstable, restarting"));
  if (!wasCalibrated && _altitude.calibrated)
    _model.logger.info().log(F("BARO ground pressure")).logln(_altitude.groundPressure);
  baro.altitudeRaw = _altitude.absolute;
  baro.altitudeBias = _altitude.groundAltitude;
  baro.altitudeGround = _altitude.height;
  baro.altitude = baro.altitudeBias + baro.altitudeGround;
  baro.altitudeBiasSamples = _altitude.calibrated ? -1 : 1;
  baro.sampleIntervalf = _altitude.sampleDt;
  baro.healthy = result == BaroAltitude::Result::READY;
  baro.updateCount++;
  baro.lastUpdateUs = now;
  baro.vario = _altitude.vario;
  baro.altitudePrev = baro.altitude;
  publish();

  if (_model.config.debug.mode == DEBUG_BARO)
  {
    _model.state.debug[0] = lrintf(baro.vario * 100.0f);     // cm/s
    _model.state.debug[1] = lrintf(baro.pressureRaw * 0.1f); // hPa x 10
    //_model.state.debug[1] = lrintf(baro.pressureRaw - 100000.0f); // Pa - 100000
    _model.state.debug[2] = lrintf(baro.temperatureRaw * 100.f); // deg C x 100
    _model.state.debug[3] = lrintf(baro.altitudeGround * 100.f); // cm
  }
}

void BaroSensor::publish()
{
  const auto& baro = _model.state.baro;
  _model.state.baro.sample.store({baro.healthy, baro.altitudeBiasSamples, baro.pressure, baro.altitudeGround,
      baro.vario, baro.sampleIntervalf, baro.updateCount, baro.lastUpdateUs});
}

} // namespace Xsfc::Sensor
