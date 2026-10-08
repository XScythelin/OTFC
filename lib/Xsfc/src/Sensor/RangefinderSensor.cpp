#include "RangefinderSensor.hpp"
#include <algorithm>

namespace Xsfc::Sensor {

RangefinderSensor::RangefinderSensor(Model& model):
  _model(model), _rangefinder(nullptr), _wait(0), _maxRangeMm(0), _isMsp(false), _lastExtCount(0) {}

int RangefinderSensor::begin()
{
  const bool msp = isMspRangefinder(_model.config.rangefinder.dev) && _model.isFeatureActive(FEATURE_RANGEFINDER);
  if (!msp && (!_model.state.rangefinder.present || !_model.state.rangefinder.dev)) return 0;

  _isMsp = msp;
  _rangefinder = _model.state.rangefinder.dev; // null for MSP source
  _lastExtCount = 0;
  _pollStarted = false;
  _lastStatus = RangeStatus::NO_DATA;
  _sampleQuality = 0;

  int rate;
  if (_isMsp)
  {
    rate = 50;
    _maxRangeMm = rangefinderProfileMaxMm(_model.config.rangefinder.dev);
  }
  else
  {
    const int interval = _rangefinder->getDelay();
    rate = 1000000 / interval;
    _maxRangeMm = _rangefinder->getMaxRangeMm();
  }
  const int32_t configuredMax = (int32_t)_model.config.rangefinder.maxRange * 10;
  if (configuredMax > 0)
    _maxRangeMm = _maxRangeMm > 0 ? std::min(configuredMax, _maxRangeMm) : configuredMax;

  _measurement.begin(_model.config.rangefinder.filter);

  _model.state.rangefinder.rate = rate;
  _model.state.rangefinder.valid = false;
  _model.state.rangefinder.distance = 0.0f;
  _model.state.rangefinder.height = 0.0f;
  _model.state.rangefinder.maxRangeMm = _maxRangeMm;
  _model.state.rangefinder.validSamples = 0;
  _model.state.rangefinder.lastSampleUs = 0;
  _model.state.rangefinder.sampleIntervalf = 0.f;
  _model.state.rangefinder.vario = 0.f;
  publish();

  const auto type = _isMsp ? (RangefinderDeviceType)_model.config.rangefinder.dev : _rangefinder->getType();
  _model.logger.info()
      .log(F("RANGEFINDER INIT"))
      .log(FPSTR(Device::RangefinderDevice::getName(type)))
      .log(rate)
      .logln(_maxRangeMm);

  return 1;
}

int RangefinderSensor::update()
{
  return read();
}

int RangefinderSensor::read()
{
  const uint32_t now = micros();
  if (_isMsp)
  {
    if (!_model.isFeatureActive(FEATURE_RANGEFINDER)) return 0;

    Xsfc::RangefinderState& rf = _model.state.rangefinder;
    const auto feed = rf.feed.load();

    if (!feed.seen || (uint32_t)(now - feed.timestamp) > RangeMeasurement::TIMEOUT_US)
    {
      rf.present = false;
      _measurement.invalidate(feed.seen ? RangeStatus::STALE : RangeStatus::NO_DATA);
      publish();
      return 0;
    }

    if (feed.count == _lastExtCount) return 0;
    _lastExtCount = feed.count;
    rf.present = true;
    _sampleQuality = feed.quality;

    return applySample(feed.raw, feed.timestamp);
  }

  if (!_rangefinder || !_model.rangefinderActive()) return 0;

  if (!_measurement.fresh(now) && _model.state.rangefinder.updateCount > 0)
  {
    _measurement.invalidate(RangeStatus::STALE);
    publish();
  }
  if (_pollStarted && (int32_t)(now - _wait) < 0) return 0;
  _pollStarted = true;
  _wait = now + _rangefinder->getDelay();

  const int32_t raw = _rangefinder->readRangeMm();
  if (raw == RANGEFINDER_NO_NEW_DATA)
  {
    // no new sample ready yet
    return 0;
  }

  return applySample(raw, now);
}

int RangefinderSensor::applySample(int32_t raw, uint32_t timestamp)
{
  Xsfc::RangefinderState& rf = _model.state.rangefinder;

  rf.raw = raw;
  _measurement.sample(raw, _model.state.attitude.cosTheta, timestamp, _maxRangeMm);
  if (!_isMsp && raw == RANGEFINDER_HARDWARE_FAILURE) _measurement.invalidate(RangeStatus::IO_ERROR);
  rf.updateCount++;
  publish();

  if (_model.config.debug.mode == DEBUG_RANGEFINDER)
  {
    _model.state.debug[0] = std::clamp(raw, (int32_t)-32000, (int32_t)32000);             // raw mm
    _model.state.debug[1] = std::clamp(lrintf(rf.height * 1000.0f), -32000l, 32000l);     // height mm
    _model.state.debug[2] = rf.valid ? 1 : 0;
  }

  return 1;
}

void RangefinderSensor::publish()
{
  auto& rf = _model.state.rangefinder;
  rf.valid = _measurement.valid;
  rf.validSamples = _measurement.streak;
  rf.distance = _measurement.distance;
  rf.height = _measurement.height;
  rf.vario = _measurement.vario;
  rf.sampleIntervalf = _measurement.sampleDt;
  rf.lastSampleUs = _measurement.lastSampleUs;
  rf.status = _measurement.status;
  if (rf.sampleIntervalf > 0.f) rf.rate = std::lround(1.f / rf.sampleIntervalf);
  RangeFrame frame;
  frame.present = rf.present;
  frame.valid = rf.valid;
  frame.validSamples = rf.validSamples;
  frame.quality = _sampleQuality;
  frame.raw = rf.raw;
  frame.maxRangeMm = rf.maxRangeMm;
  frame.rate = rf.rate;
  frame.height = rf.height;
  frame.distance = rf.distance;
  frame.vario = rf.vario;
  frame.sampleIntervalf = rf.sampleIntervalf;
  frame.lastSampleUs = rf.lastSampleUs;
  frame.updateCount = rf.updateCount;
  frame.status = rf.status;
  rf.sample.store(frame);
  if (_lastStatus != _measurement.status)
  {
    if (_measurement.status == RangeStatus::VALID)
      _model.logger.info().logln(F("RANGEFINDER samples valid"));
    else
      _model.logger.err().log(F("RANGEFINDER")).logln(rangeStatusName(_measurement.status));
    _lastStatus = _measurement.status;
  }
}

} // namespace Xsfc::Sensor
