#include "Connect/MspParser.hpp"
#include "Connect/SensorMsp.hpp"
#include "Control/HoldSafety.hpp"
#include "Control/HorizontalHold.hpp"
#include "Control/RangeAltitude.hpp"
#include "Device/Rangefinder/RangefinderVL53L0X.hpp"
#include "Sensor/FlowMeasurement.hpp"
#include "Sensor/RangeMeasurement.hpp"
#include "Sensor/SensorFrames.hpp"
#include "Utils/Crc.hpp"
#include "Utils/Snapshot.hpp"
#include <functional>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

using namespace Xsfc;

namespace {

void require(bool condition, const char* message)
{
  if (!condition) throw std::runtime_error(message);
}

void near(float actual, float expected, float tolerance, const char* message)
{
  if (!std::isfinite(actual) || std::fabs(actual - expected) > tolerance)
    throw std::runtime_error(std::string(message) + ": " + std::to_string(actual) + " != " + std::to_string(expected));
}

std::vector<uint8_t> frame(uint16_t command, const std::vector<uint8_t>& payload, bool wrapped = false)
{
  std::vector<uint8_t> body{0, (uint8_t)command, (uint8_t)(command >> 8),
      (uint8_t)payload.size(), (uint8_t)(payload.size() >> 8)};
  body.insert(body.end(), payload.begin(), payload.end());
  body.push_back(Utils::crc8_dvb_s2(0, body.data(), body.size()));
  std::vector<uint8_t> result{'$', (uint8_t)(wrapped ? 'M' : 'X'), '<'};
  if (wrapped)
  {
    result.push_back((uint8_t)body.size());
    result.push_back(255);
  }
  result.insert(result.end(), body.begin(), body.end());
  if (wrapped) result.push_back(Utils::crc8_xor(0, result.data() + 3, result.size() - 3));
  return result;
}

Connect::MspMessage parse(const std::vector<uint8_t>& bytes)
{
  Connect::MspParser parser;
  Connect::MspMessage message;
  for (uint8_t byte : bytes) parser.parse((char)byte, message);
  return message;
}

bool flowSample(Sensor::FlowMeasurement& flow, float x, float y, uint8_t quality = 255,
    float height = 2.f, float yaw = 0.f, float gyroX = 0.f, float gyroY = 0.f,
    uint32_t interval = 20000)
{
  return flow.sample(x, y, quality, interval, 1.f / 200.f, ALIGN_CW0_DEG,
      1.f, 1.f, gyroX, gyroY, true, true, true, height, 0.08f, yaw);
}

struct VlBus : Device::BusDevice
{
  BusType getType() const override { return BUS_I2C; }
  int8_t read(uint8_t, uint8_t reg, uint8_t length, uint8_t* output) override
  {
    if (failed) return 0;
    if (reg == 0x13 && length == 1) output[0] = ready ? 7 : 0;
    else if (reg == 0x1E && length == 2)
    {
      output[0] = range >> 8;
      output[1] = range & 255;
    }
    else return 0;
    return length;
  }
  int8_t readFast(uint8_t address, uint8_t reg, uint8_t length, uint8_t* output) override
  {
    return read(address, reg, length, output);
  }
  bool write(uint8_t, uint8_t, uint8_t, const uint8_t*) override { return !failedWrite; }
  bool ready = true, failed = false, failedWrite = false;
  uint16_t range = 1800;
};

} // namespace

int runNavigationTests()
{
  int count = 0, failures = 0;
  const auto test = [&](const char* name, const std::function<void()>& run)
  {
    ++count;
    try
    {
      run();
      std::cout << "PASS " << name << '\n';
    }
    catch (const std::exception& error)
    {
      ++failures;
      std::cerr << "FAIL " << name << ": " << error.what() << '\n';
    }
  };

  test("Navigation filters initialize without a nonzero measurement transient", []()
  {
    for (const auto type : {FILTER_PT1, FILTER_PT2, FILTER_PT3, FILTER_BIQUAD,
        FILTER_FO, FILTER_FIR2, FILTER_MEDIAN3, FILTER_NONE})
    {
      Utils::Filter filter;
      filter.begin(FilterConfig(type, 10), 100);
      filter.reset(2.5f);
      for (int i = 0; i < 20; ++i)
        near(filter.update(2.5f), 2.5f, 0.0001f, "seeded filter transient");
    }
  });
  test("MSPv2 accepts the entire unsigned CRC byte range", []()
  {
    int highCrc = 0;
    for (int quality = 0; quality < 256; ++quality)
    {
      const auto bytes = frame(Connect::MSP2_SENSOR_RANGEFINDER, {(uint8_t)quality, 0xC4, 9, 0, 0});
      if (bytes.back() >= 128) ++highCrc;
      auto message = parse(bytes);
      require(message.isReady(), "valid native MSPv2 packet rejected");
      Connect::RangePacket packet{};
      require(Connect::readRangePacket(message, packet), "range payload not decoded");
      require(packet.distanceMm == 2500 && packet.quality == quality, "range payload changed");
    }
    require(highCrc > 100, "high-bit CRC values not tested");
  });
  test("MSPv2-in-v1 and native flow packets decode identically", []()
  {
    const std::vector<uint8_t> payload{200, 0xFF, 0xFF, 0xFF, 0xFF, 0, 0, 0, 0x80};
    for (bool wrapped : {false, true})
    {
      auto message = parse(frame(Connect::MSP2_SENSOR_OPTIC_FLOW, payload, wrapped));
      require(message.isReady() && message.cmd == Connect::MSP2_SENSOR_OPTIC_FLOW, "wrapped message rejected");
      Connect::FlowPacket packet{};
      require(Connect::readFlowPacket(message, packet), "flow decode failed");
      require(packet.motionX == -1 && packet.motionY == INT32_MIN && packet.quality == 200, "signed flow decode wrong");
    }
  });
  test("CRC corruption and short sensor payloads never refresh samples", []()
  {
    auto bytes = frame(Connect::MSP2_SENSOR_RANGEFINDER, {255, 0xFF, 0xFF, 0xFF, 0xFF});
    bytes.back() ^= 1;
    require(!parse(bytes).isReady(), "corrupt CRC accepted");
    auto message = parse(frame(Connect::MSP2_SENSOR_RANGEFINDER, {1, 2, 3}));
    Connect::RangePacket packet{42, 123};
    require(!Connect::readRangePacket(message, packet), "short range accepted");
    require(message.read == 0 && packet.distanceMm == 123, "short payload consumed data");
    auto flow = parse(frame(Connect::MSP2_SENSOR_OPTIC_FLOW, {1, 2, 3, 4, 5}));
    Connect::FlowPacket flowPacket{};
    require(!Connect::readFlowPacket(flow, flowPacket), "short flow accepted");
  });
  test("sensor reply-direction frames parse and unrelated replies remain distinguishable", []()
  {
    auto bytes = frame(Connect::MSP2_SENSOR_RANGEFINDER, {0, 0xD0, 7, 0, 0});
    bytes[2] = '>';
    const auto message = parse(bytes);
    require(message.isReady() && !message.isCmd() && Connect::isSensorFeed(message.cmd), "push reply direction rejected");
    require(!Connect::isSensorFeed(101), "ordinary MSP reply mistaken for sensor push");
  });
  test("preserved mode IDs and dense MSP flags include new modes", []()
  {
    require(MODE_GPIO_OUTPUT == 9 && MODE_RANGEHOLD == 13 && MODE_POSHOLD == 14, "saved mode IDs changed");
    require(!isKnownFlightMode(8) && !isKnownFlightMode(11) && !isKnownFlightMode(12), "retired AUX IDs still active");
    require(mspModeMask(modeBit(MODE_GPIO_OUTPUT) | modeBit(MODE_POSHOLD)) == ((1u << 8) | (1u << 10)),
        "MSP status disagrees with BOXNAMES order");
    require(sizeof(FLIGHT_MODES) / sizeof(FLIGHT_MODES[0]) == 12, "mode metadata mismatch");
    require(RANGE_HOLD_MODES & modeBit(MODE_POSHOLD), "POSHOLD lacks automatic height");
    require(mspModeMask((1u << 11) | (1u << 12)) == 0, "retired flags advertised");
  });
  test("MSP range profiles do not clip all sensors at one metre", []()
  {
    require(rangefinderProfileMaxMm(RANGEFINDER_MTF02) == 2500, "MTF02 working range");
    require(rangefinderProfileMaxMm(RANGEFINDER_MTF02P) == 6000, "MTF02P working range");
    require(rangefinderProfileMaxMm(RANGEFINDER_MSP) == 0, "generic MSP still has a fixed cap");
    Sensor::RangeMeasurement range;
    range.begin(FilterConfig(FILTER_PT1, 10));
    require(range.sample(2400, 1.f, 10000, 2500), "MTF02 reading above 1m rejected");
    require(range.sample(5500, 1.f, 30000, 6000), "MTF02P reading above 1.5m rejected");
    require(range.sample(8000, 1.f, 50000, 0), "generic MSP longer range rejected");
    require(!range.sample(2600, 1.f, 70000, 2500) && range.status == Sensor::RangeStatus::LIMIT, "device cap ignored");
  });
  test("range readiness needs three samples and survives micros wrap", []()
  {
    Sensor::RangeMeasurement range;
    range.begin(FilterConfig(FILTER_PT1, 10));
    uint32_t now = UINT32_MAX - 10000;
    range.sample(1000, 1.f, now, 2000);
    require(!range.ready(now), "one sample marked ready");
    now += 20000;
    range.sample(1000, 1.f, now, 2000);
    now += 20000;
    range.sample(1000, 1.f, now, 2000);
    require(range.ready(now), "ready state lost across timestamp wrap");
    require(!range.ready(now + 200001), "stale measurement remains ready");
    near(range.sampleDt, 0.02f, 0.00001f, "wrapped range dt");
  });
  test("invalid range and tilt do not feed zero into distance filter", []()
  {
    Sensor::RangeMeasurement range;
    range.begin(FilterConfig(FILTER_BIQUAD, 10));
    range.sample(2000, 1.f, 10000, 0);
    near(range.height, 2.f, 0.0001f, "first range must seed filter");
    require(!range.sample(-1, 1.f, 30000, 0), "invalid negative range accepted");
    near(range.height, 2.f, 0.0001f, "invalid sample polluted height");
    require(!range.sample(2000, 0.8f, 50000, 0), "excessive tilt accepted");
    range.sample(3000, 1.f, 70000, 0);
    near(range.height, 3.f, 0.0001f, "recovery filter was not reseeded");
    near(range.vario, 0.f, 0.0001f, "recovery derivative spike");
  });
  test("range vario follows irregular intervals and tilt-compensated height", []()
  {
    Sensor::RangeMeasurement range;
    range.begin(FilterConfig(FILTER_PT1, 10));
    uint32_t now = 10000;
    const uint32_t start = now;
    for (int i = 0; i < 250; ++i)
    {
      now += i % 2 ? 15000 : 35000;
      const float height = 1.f + (now - start) * 1e-6f;
      range.sample(std::lround(height * 1000.f / 0.95f), 0.95f, now, 0);
    }
    near(range.vario, 1.f, 0.03f, "actual-time range vario");
    near(range.height, 1.f + (now - start) * 1e-6f, 0.04f, "vertical range height");
    now += 300000;
    range.sample(7000, 1.f, now, 0);
    near(range.vario, 0.f, 0.0001f, "range gap derivative");
  });
  test("VL53L0X distinguishes no-data, out-of-range and read/write failures", []()
  {
    VlBus bus;
    Device::Rangefinder::RangefinderVL53L0X driver;
    driver.setBus(&bus, 0x29);
    require(driver.readRangeMm() == 1800, "VL53L0X distance");
    bus.ready = false;
    require(driver.readRangeMm() == RANGEFINDER_NO_NEW_DATA, "pending sample status");
    bus.ready = true;
    bus.range = 8191;
    require(driver.readRangeMm() == RANGEFINDER_OUT_OF_RANGE, "out of range mistaken for no sample");
    bus.failed = true;
    require(driver.readRangeMm() == RANGEFINDER_HARDWARE_FAILURE, "short read failure ignored");
    bus.failed = false;
    bus.range = 1000;
    bus.failedWrite = true;
    require(driver.readRangeMm() == RANGEFINDER_HARDWARE_FAILURE, "interrupt clear failure ignored");
  });
  test("MTF flow units and axis cross-coupling match linear velocity", []()
  {
    Sensor::FlowMeasurement flow;
    flow.begin(FilterConfig(FILTER_NONE, 0));
    require(flowSample(flow, 4.f, -2.f), "flow sample rejected");
    near(flow.rateX, 1.f, 0.0001f, "MTF counts to rad/s");
    near(flow.forward, 1.f, 0.0001f, "forward flow axis");
    near(flow.right, 2.f, 0.0001f, "right flow axis");
  });
  test("Matek INAV scale and downward alignment are applied once", []()
  {
    Sensor::FlowMeasurement flow;
    flow.begin(FilterConfig(FILTER_NONE, 0));
    require(flow.sample(21.f, 0.f, 255, 20000, Utils::toRad(1.f) / 10.5f,
        ALIGN_CW0_DEG_FLIP, 1.f, 1.f, 0.f, 0.f, true, true, true, 2.f, 0.08f, 0.f), "Matek rejected");
    near(flow.rateX, -Utils::toRad(100.f), 0.0001f, "Matek scale/alignment");
  });
  test("rotation-only motion cancels with averaged gyro", []()
  {
    Sensor::FlowMeasurement flow;
    flow.begin(FilterConfig(FILTER_NONE, 0));
    flowSample(flow, 4.f, -2.f, 255, 2.f, 0.f, 1.f, -0.5f);
    near(flow.forward, 0.f, 0.00001f, "pitch rotation cancellation");
    near(flow.right, 0.f, 0.00001f, "roll rotation cancellation");
  });
  test("flow quality hysteresis and missing range invalidate position", []()
  {
    Sensor::FlowMeasurement flow;
    flow.begin(FilterConfig(FILTER_NONE, 0));
    require(!flowSample(flow, 0.f, 0.f, 70), "quality hysteresis entry threshold ignored");
    require(flowSample(flow, 0.f, 0.f, 100), "high quality not accepted");
    require(flowSample(flow, 0.f, 0.f, 50), "hysteresis did not retain valid quality");
    require(!flowSample(flow, 0.f, 0.f, 20), "low quality not rejected");
    require(!flowSample(flow, 0.f, 0.f, 255, 0.02f), "too-close floor accepted");
    require(!flowSample(flow, 0.f, 0.f, 255, 2.f, 0.f, 0.f, 0.f, 0), "first flow interval accepted");
  });
  test("earth velocity is yaw invariant and transforms tilt", []()
  {
    Sensor::FlowMeasurement flow;
    flow.begin(FilterConfig(FILTER_NONE, 0));
    flowSample(flow, 0.f, -4.f, 255, 1.f, Utils::pi() / 2.f);
    near(flow.velocityX, 0.f, 0.0001f, "yaw transformed X");
    near(flow.velocityY, 1.f, 0.0001f, "yaw transformed Y");
    flow.sample(0.f, -4.f, 255, 20000, 1.f / 200.f, ALIGN_CW0_DEG,
        1.f, 1.f, 0.f, 0.f, true, true, true, 1.f, 0.08f, 0.f, 0.f, 0.2f, 0.5f);
    near(flow.velocityX, std::cos(0.2f) + 0.5f * std::sin(0.2f), 0.0001f, "body-to-earth tilt projection");
  });
  test("gyro history averages the integration window, including delay and wrap", []()
  {
    Sensor::FlowGyroWindow gyro;
    uint32_t now = UINT32_MAX - 100000;
    gyro.add(1.f, -2.f, now);
    for (int i = 0; i < 300; ++i)
    {
      now += 1000;
      gyro.add(1.f, -2.f, now);
    }
    float x = 0.f, y = 0.f;
    require(gyro.average(now - 65000, 20000, x, y), "delayed wrapped gyro window unavailable");
    near(x, 1.f, 0.0001f, "gyro X average");
    near(y, -2.f, 0.0001f, "gyro Y average");
    require(!gyro.average(now + 100000, 20000, x, y), "future gyro window accepted");
    now += 100000;
    gyro.add(1.f, 1.f, now);
    require(!gyro.average(now, 20000, x, y), "gyro gap retained stale integration");
  });
  test("range hold works without a barometer and latches loss until mode reset", []()
  {
    Control::HoldSafety safety;
    const uint32_t rangeMode = modeBit(MODE_RANGEHOLD);
    require(safety.update(true, rangeMode, rangeMode, false, true, true, false), "barometer required by range hold");
    require(!safety.update(true, rangeMode, rangeMode, true, false, false, true), "loss did not return manual throttle");
    require(safety.rangeLost, "range loss not latched");
    require(!safety.update(true, rangeMode, rangeMode, true, true, true, false), "automatic reengagement after sensor recovery");
    safety.update(true, 0, 0, true, true, true, false);
    require(safety.update(true, rangeMode, rangeMode, false, true, true, false), "mode reset did not release latch");
  });
  test("POSHOLD has range priority and ordinary ALTHOLD ignores range loss", []()
  {
    Control::HoldSafety safety;
    const uint32_t flow = modeBit(MODE_POSHOLD);
    require(safety.update(true, flow | BARO_HOLD_MODES, flow | BARO_HOLD_MODES, false, true, true, false),
        "POSHOLD acquired barometer dependency");
    require(safety.rangeSource, "range source priority missing");
    require(safety.update(true, modeBit(MODE_ALTHOLD), modeBit(MODE_ALTHOLD), true, false, true, false),
        "range loss disabled ordinary ALTHOLD");
    require(!safety.rangeSource && !safety.rangeLost, "range state leaked into barometer hold");
    require(!safety.update(false, flow, flow, true, true, true, false), "disarmed automatic throttle enabled");
  });
  test("range and position hold transitions preserve vertical state", []()
  {
    Control::HoldSafety safety;
    const uint32_t range = modeBit(MODE_RANGEHOLD), position = modeBit(MODE_POSHOLD);
    bool engaged = false, source = false;
    for (uint32_t modes : {range, range | position, position, range | position, range})
    {
      require(safety.update(true, modes, modes, false, true, true, engaged), "range transition rejected");
      require(Control::HoldSafety::needsReset(engaged, source, safety.rangeSource) == !engaged,
          "same-source transition resets vertical target/trim");
      source = safety.rangeSource;
      engaged = true;
    }
    require(Control::HoldSafety::needsReset(true, true, false), "source change did not reset");
    require(!safety.update(true, range, range, false, false, true, true), "range loss accepted");
    require(!safety.update(true, position, position, false, true, true, false), "mode change cleared loss latch");
    require(!safety.update(false, position, position, false, false, true, true), "disarmed hold accepted");
    require(!safety.rangeLost, "disarm re-latched range loss");
    safety.update(true, 0, 0, false, true, true, false);
    require(safety.update(true, position, position, false, true, true, false), "off/on did not recover");
  });

  test("flow rejects nonfinite gyro even when window is marked valid", []()
  {
    Sensor::FlowMeasurement flow;
    flow.begin(FilterConfig(FILTER_NONE, 0));
    require(!flowSample(flow, 0.f, 0.f, 255, 2.f, 0.f, NAN, 0.f), "NaN gyro accepted");
    require(flow.status == Sensor::FlowStatus::GYRO && !flow.valid && flow.streak == 0,
        "invalid gyro published as usable flow");
    require(flowSample(flow, 0.f, 0.f), "finite flow failed recovery");
    require(flow.streak == 1, "recovery skipped readiness streak");
  });

  test("Matek and MTF scales give matching body velocity for equal angular motion", []()
  {
    for (float scale : {Utils::toRad(1.f) / 10.5f, 1.f / 200.f})
    {
      Sensor::FlowMeasurement flow;
      flow.begin(FilterConfig(FILTER_NONE, 0));
      const float dt = 0.02f, angularRate = 0.5f;
      const float counts = angularRate * dt / scale;
      require(flow.sample(counts, -counts, 255, 20000, scale, ALIGN_CW0_DEG,
          1.f, 1.f, 0.f, 0.f, true, true, true, 2.f, 0.08f, 0.f), "scaled flow rejected");
      near(flow.velocityX, 1.f, 0.0001f, "forward sign/scale");
      near(flow.velocityY, 1.f, 0.0001f, "right sign/scale");
      require(flow.sample(counts, -counts, 255, 20000, scale, ALIGN_CW0_DEG,
          1.f, 1.f, angularRate, -angularRate, true, true, true, 2.f, 0.08f, 0.f), "rotation sample rejected");
      near(flow.velocityX, 0.f, 0.0001f, "pitch rotation not cancelled");
      near(flow.velocityY, 0.f, 0.0001f, "roll rotation not cancelled");
      require(!flow.sample(counts, -counts, 255, 20000, scale, ALIGN_CW0_DEG,
          1.f, 1.f, 0.f, 0.f, true, true, false, 2.f, 0.08f, 0.f), "range failure did not invalidate flow");
    }
  });

  test("flow failure preserves range throttle and range failure latches both-mode height loss", []()
  {
    Control::HoldSafety safety;
    const uint32_t modes = modeBit(MODE_RANGEHOLD) | modeBit(MODE_POSHOLD);
    Sensor::RangeFrame range;
    range.present = range.valid = true;
    range.validSamples = 3;
    range.height = 2.f;
    range.lastSampleUs = 100000;
    Sensor::FlowFrame flow;
    flow.present = flow.valid = true;
    flow.validSamples = 3;
    flow.lastSampleUs = 100000;
    require(Sensor::flowFrameReady(flow, 100000), "healthy flow not ready");
    require(safety.update(true, modes, modes, false, Sensor::rangeFrameReady(range, 100000), true, false),
        "healthy range throttle rejected");
    flow.valid = false;
    require(!Sensor::flowFrameReady(flow, 100000), "failed flow still ready");
    require(safety.update(true, modes, modes, false, Sensor::rangeFrameReady(range, 100000), true, true),
        "flow-only loss disabled range throttle");
    require(!safety.update(true, modes, modes, false, Sensor::rangeFrameReady(range, 300001), true, true),
        "stale range kept automatic throttle");
    range.lastSampleUs = 300001;
    require(!safety.update(true, modes, modes, false, Sensor::rangeFrameReady(range, 300001), true, false),
        "range recovery bypassed loss latch");
  });

  test("rejected range request cannot steal the active barometric source", []()
  {
    Control::HoldSafety safety;
    const uint32_t baro = modeBit(MODE_ALTHOLD);
    const uint32_t range = modeBit(MODE_RANGEHOLD);
    require(safety.update(true, baro, baro | range, true, false, true, false),
        "inactive range request disabled ALTHOLD");
    require(!safety.rangeSource, "rejected range selected as source");
    require(safety.update(true, baro | range, baro | range, true, true, true, false),
        "accepted range failed to engage");
    require(safety.rangeSource, "accepted range lost priority");
    require(!safety.update(true, baro | range, baro | range, true, false, true, true),
        "range failure silently switched source");
    require(safety.rangeLost && safety.rangeSource, "range failure not latched");
  });

  test("range estimator holds absolute surface distance without ground zeroing", []()
  {
    Control::RangeAltitude altitude;
    altitude.update(false, true, true, 0.18f, 0.f, 0.02f, true, 0.f, 0.002f, 0.1f);
    near(altitude.motion.height, 0.18f, 0.0001f, "range zero incorrectly subtracted landing gear height");
    altitude.update(true, true, true, 2.f, 0.f, 0.02f, true, 0.1f, 0.002f, 0.1f);
    near(altitude.motion.height, 2.f, 0.0001f, "independent range initialization");
    altitude.update(true, false, false, 0.f, 0.f, 0.f, true, 0.1f, 0.002f, 0.1f);
    require(!altitude.ready, "lost range still ready");
    altitude.update(true, true, true, 4.f, 0.f, 0.f, true, 0.1f, 0.002f, 0.1f);
    near(altitude.motion.height, 4.f, 0.0001f, "range recovery failed to reset");
  });
  test("horizontal controller brakes in the right direction at different yaw angles", []()
  {
    Control::HorizontalHold hold;
    hold.begin(0.65f, 2.f, Utils::toRad(20.f), 2.f, 0.15f, 1.f, 0.4f, 500);
    hold.reset(1.f, 0.f);
    hold.update(1.f, 0.f, 0.f);
    require(hold.anglePitch < 0.f, "forward drift accelerated instead of braked");
    hold.reset(0.f, 1.f);
    hold.update(0.f, 1.f, 0.f);
    require(hold.angleRoll > 0.f, "rightward drift accelerated instead of braked");
    hold.reset(1.f, 0.f);
    hold.update(1.f, 0.f, Utils::pi() / 2.f);
    require(hold.angleRoll < 0.f, "world anchor did not rotate corrective lean with yaw");
  });
  test("position hold closed loop returns to anchor while yaw changes", []()
  {
    Control::HorizontalHold hold;
    const float limit = Utils::toRad(20.f);
    hold.begin(0.65f, 2.f, limit, 2.f, 0.15f, 1.f, 0.4f, 500);
    float x = 0.f, y = 0.f, vx = 0.6f, vy = -0.3f;
    hold.reset(vx, vy);
    for (int i = 0; i < 15000; ++i)
    {
      const float yaw = i > 7500 ? Utils::pi() / 2.f : 0.f;
      if (i % 10 == 0) hold.sample(vx, vy, 0.02f);
      hold.update(vx, vy, yaw);
      require(std::fabs(hold.angleRoll) <= limit + 0.0001f && std::fabs(hold.anglePitch) <= limit + 0.0001f,
          "unbounded automatic bank angle");
      const float forward = 9.80665f * std::tan(hold.anglePitch);
      const float right = -std::sqrt(9.80665f * 9.80665f + forward * forward) * std::tan(hold.angleRoll);
      const float ax = std::cos(yaw) * forward - std::sin(yaw) * right;
      const float ay = std::sin(yaw) * forward + std::cos(yaw) * right;
      x += vx * 0.002f + 0.5f * ax * 0.002f * 0.002f;
      y += vy * 0.002f + 0.5f * ay * 0.002f * 0.002f;
      vx += ax * 0.002f;
      vy += ay * 0.002f;
    }
    near(x, 0.f, 0.15f, "anchor X after yaw change");
    near(y, 0.f, 0.15f, "anchor Y after yaw change");
    near(vx, 0.f, 0.1f, "held horizontal speed");
    near(vy, 0.f, 0.1f, "held horizontal speed");
    hold.reset(vx, vy);
    near(hold.positionX, 0.f, 0.f, "pilot release did not capture new anchor");
  });
  test("barometer freshness and finite checks remain independent", []()
  {
    Sensor::BaroFrame frame;
    frame.healthy = true;
    frame.altitudeBiasSamples = -1;
    frame.pressure = 101325.f;
    frame.lastUpdateUs = UINT32_MAX - 10000;
    require(Sensor::baroFrameReady(frame, 10000), "barometer timestamp wrap");
    require(!Sensor::baroFrameReady(frame, 300000), "stale barometer accepted");
    frame.vario = NAN;
    require(!Sensor::baroFrameReady(frame, 10000), "NaN barometer accepted");
  });
  test("cross-core sensor snapshots cannot mix fields from different samples", []()
  {
    struct Pair { uint32_t value = 0; uint32_t complement = UINT32_MAX; };
    Utils::Snapshot<Pair> snapshot;
    std::atomic<bool> finished{false};
    std::thread writer([&]()
    {
      for (uint32_t i = 0; i < 30000; ++i) snapshot.store({i, ~i});
      finished.store(true);
    });
    bool coherent = true;
    do
    {
      const auto pair = snapshot.load();
      coherent = coherent && pair.complement == ~pair.value;
    } while (!finished.load());
    writer.join();
    require(coherent, "torn sensor frame");
    Utils::Snapshot<Pair> copy(snapshot);
    require(copy.load().value == snapshot.load().value, "snapshot copy compatibility");
  });

  std::cout << count - failures << "/" << count << " navigation tests passed\n";
  return failures;
}
