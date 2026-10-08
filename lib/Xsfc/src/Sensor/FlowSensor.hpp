#pragma once

#include "Model.h"
#include "FlowMeasurement.hpp"

namespace Xsfc::Sensor {

class FlowSensor
{
public:
  explicit FlowSensor(Model& model): _model(model) {}

  int begin()
  {
    _measurement.begin(_model.config.posHold.velFilter);
    _gyro.reset();
    _lastCount = 0;
    _lastStatus = FlowStatus::NO_DATA;
    _model.state.flow.present = _model.state.flow.valid = false;
    _model.state.flow.validSamples = 0;
    _model.state.flow.processedCount = 0;
    _processedFeed = {};
    _model.state.flow.sample.store({});
    return 1;
  }

  void accumulateGyro()
  {
    if (_model.isFeatureActive(FEATURE_RANGEFINDER))
      _gyro.add(_model.state.gyro.adc.x, _model.state.gyro.adc.y, micros());
  }

  int update()
  {
    auto& flow = _model.state.flow;
    const uint32_t now = micros();
    const auto feed = flow.feed.load();
    if (!_model.isFeatureActive(FEATURE_RANGEFINDER) || !feed.seen ||
        (uint32_t)(now - feed.timestamp) > 200000)
    {
      flow.present = false;
      _measurement.invalidate(feed.seen ? FlowStatus::STALE : FlowStatus::NO_DATA);
      publish();
      return 0;
    }
    if (feed.count == _lastCount) return 0;
    _lastCount = feed.count;
    const auto& cfg = _model.config.navigation;
    const auto& ph = _model.config.posHold;
    const bool mtf = cfg.flowDev == FLOW_MTF || (cfg.flowDev == FLOW_AUTO &&
        (_model.config.rangefinder.dev == RANGEFINDER_MTF02 || _model.config.rangefinder.dev == RANGEFINDER_MTF02P));
    const float scale = mtf ? 1.f / 200.f :
        (cfg.flowScale > 0 ? Utils::toRad(1.f) / (cfg.flowScale * 0.01f) : 0.f);
    float gyroX = 0.f, gyroY = 0.f;
    const bool gyroValid = _gyro.average(feed.timestamp - cfg.flowDelayMs * 1000u, feed.interval, gyroX, gyroY);
    const auto range = _model.rangeSample();
    _measurement.sample(feed.motionX, feed.motionY, feed.quality, feed.interval, scale,
        cfg.flowAlign, ph.flowGainX * 0.01f, ph.flowGainY * 0.01f, gyroX, gyroY, gyroValid,
        ph.useGyroComp, rangeFrameReady(range, now), range.height,
        cfg.flowMinRangeMm * 0.001f, _model.state.attitude.euler.z,
        _model.state.attitude.euler.x, _model.state.attitude.euler.y, range.vario);
    flow.present = true;
    flow.lastSampleUs = feed.timestamp;
    _processedFeed = feed;
    ++flow.processedCount;
    publish();
    return 1;
  }

private:
  void publish()
  {
    auto& flow = _model.state.flow;
    flow.valid = _measurement.valid;
    flow.validSamples = _measurement.streak;
    flow.status = (uint8_t)_measurement.status;
    flow.sampleIntervalf = _measurement.sampleDt;
    flow.flowRateX = _measurement.rateX;
    flow.flowRateY = _measurement.rateY;
    flow.bodyRateX = _measurement.bodyX;
    flow.bodyRateY = _measurement.bodyY;
    flow.velocityForward = _measurement.forward;
    flow.velocityRight = _measurement.right;
    flow.velocityX = _measurement.velocityX;
    flow.velocityY = _measurement.velocityY;
    if (flow.sampleIntervalf > 0.f) flow.rate = std::lround(1.f / flow.sampleIntervalf);
    FlowFrame frame;
    frame.present = flow.present;
    frame.valid = flow.valid;
    frame.validSamples = flow.validSamples;
    frame.quality = _processedFeed.quality;
    frame.motionX = _processedFeed.motionX;
    frame.motionY = _processedFeed.motionY;
    frame.rate = flow.rate;
    frame.sampleIntervalf = flow.sampleIntervalf;
    frame.flowRateX = flow.flowRateX;
    frame.flowRateY = flow.flowRateY;
    frame.bodyRateX = flow.bodyRateX;
    frame.bodyRateY = flow.bodyRateY;
    frame.velocityForward = flow.velocityForward;
    frame.velocityRight = flow.velocityRight;
    frame.velocityX = flow.velocityX;
    frame.velocityY = flow.velocityY;
    frame.lastSampleUs = flow.lastSampleUs;
    frame.processedCount = flow.processedCount;
    frame.status = _measurement.status;
    flow.sample.store(frame);
    if (_lastStatus != _measurement.status)
    {
      if (_measurement.status == FlowStatus::VALID)
        _model.logger.info().logln(F("OPTIC FLOW samples valid"));
      else if (_measurement.status != FlowStatus::NO_DATA)
        _model.logger.err().log(F("OPTIC FLOW")).logln(flowStatusName(_measurement.status));
      _lastStatus = _measurement.status;
    }
  }

  Model& _model;
  FlowMeasurement _measurement;
  FlowGyroWindow _gyro;
  uint32_t _lastCount = 0;
  FlowStatus _lastStatus = FlowStatus::NO_DATA;
  FlowFeed _processedFeed;
};

} // namespace Xsfc::Sensor
