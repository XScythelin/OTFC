#include "Device/Baro/BaroMS5611.hpp"
#include "Sensor/BaroAltitude.hpp"
#include "Control/VerticalMotion.hpp"
#include "Control/AltHoldStick.hpp"
#include "Control/AltHoldThrottle.hpp"
#include "Target/Target.h"
#include "Device/BusSPI.h"
#include <array>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

using namespace Xsfc;

int runNavigationTests();

static void require(bool condition, const char* message)
{
  if (!condition) throw std::runtime_error(message);
}

static void near(float actual, float expected, float tolerance, const char* message)
{
  if (!std::isfinite(actual) || std::fabs(actual - expected) > tolerance)
    throw std::runtime_error(std::string(message) + ": actual=" + std::to_string(actual) + ", expected=" + std::to_string(expected));
}

static uint8_t promCrc(std::array<uint16_t, 8> prom)
{
  prom[7] &= 0xFF00;
  uint16_t crc = 0;
  for (uint16_t word : prom)
  {
    for (int shift : {8, 0})
    {
      crc ^= (word >> shift) & 255;
      for (int bit = 0; bit < 8; ++bit)
        crc = (crc & 0x8000) ? (crc << 1) ^ 0x3000 : crc << 1;
    }
  }
  return (crc >> 12) & 15;
}

class FakeBus : public Device::BusDevice
{
public:
  FakeBus()
  {
    prom[7] = promCrc(prom);
    onError = [this]() { ++errors; };
  }
  BusType getType() const override { return spi ? BUS_SPI : BUS_I2C; }
  int8_t read(uint8_t, uint8_t, uint8_t, uint8_t*) override
  {
    throw std::runtime_error("MS5611 must use unmodified command transactions");
  }
  int8_t readFast(uint8_t addr, uint8_t cmd, uint8_t length, uint8_t* out) override
  {
    return read(addr, cmd, length, out);
  }
  bool write(uint8_t, uint8_t, uint8_t, const uint8_t*) override
  {
    throw std::runtime_error("MS5611 must use unmodified command transactions");
  }
  bool writeCommand(uint8_t addr, uint8_t cmd) override
  {
    addresses.push_back(addr);
    if (failCommand || (!spi && addr != address))
    {
      if (onError) onError();
      return false;
    }
    command = cmd;
    return true;
  }
  int8_t readCommand(uint8_t, uint8_t cmd, uint8_t length, uint8_t* out) override
  {
    if (shortRead)
    {
      if (onError) onError();
      return 0;
    }
    if (cmd >= 0xA0 && length == 2)
    {
      const uint16_t word = prom[(cmd - 0xA0) / 2];
      out[0] = word >> 8;
      out[1] = word & 255;
      return 2;
    }
    require(cmd == 0 && length == 3, "wrong ADC command");
    const uint32_t adc = command == 0x58 ? d2 : d1;
    out[0] = adc >> 16;
    out[1] = (adc >> 8) & 255;
    out[2] = adc & 255;
    return 3;
  }
  std::array<uint16_t, 8> prom{0, 40127, 36924, 23317, 23282, 33464, 28312, 0};
  std::vector<uint8_t> addresses;
  uint32_t d1 = 9085466;
  uint32_t d2 = 8569150;
  uint8_t address = 0x76;
  uint8_t command = 0;
  bool spi = false;
  bool shortRead = false;
  bool failCommand = false;
  int errors = 0;
};

static std::pair<float, float> measure(Device::Baro::BaroMS5611& driver)
{
  driver.setMode(BARO_MODE_TEMP);
  const float temperature = driver.readTemperature();
  driver.setMode(BARO_MODE_PRESS);
  return {driver.readPressure(), temperature};
}

static float pressureForHeight(float height)
{
  return 101325.f * std::pow(1.f - height / 44330.f, 1.f / 0.190295f);
}

static void calibrate(Sensor::BaroAltitude& estimator, uint32_t& now)
{
  for (int i = 0; i <= 100; ++i)
  {
    estimator.update(101325.f, now, false);
    now += 20000;
  }
  require(estimator.calibrated, "stable two-second calibration must finish");
}

static Control::Pid velocityPid()
{
  Control::Pid pid;
  pid.rate = 500.f;
  pid.Kp = 100 * VEL_PTERM_SCALE;
  pid.Ki = 50 * VEL_ITERM_SCALE;
  pid.Kd = 10 * VEL_DTERM_SCALE;
  pid.Kf = 0;
  pid.ftermDerivative = false;
  pid.iLimitLow = -1.f;
  pid.iLimitHigh = 1.f;
  pid.ptermFilter.begin(FilterConfig(FILTER_PT1, 5), 500);
  pid.dtermFilter.begin(FilterConfig(FILTER_PT1, 5), 500);
  pid.begin();
  return pid;
}

int main()
{
  int failures = 0;
  int count = 0;
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

  test("MS5611 datasheet vector", []()
  {
    FakeBus bus;
    Device::Baro::BaroMS5611 driver;
    require(driver.begin(&bus), "detection failed");
    const auto result = measure(driver);
    near(result.first, 100009.f, 1.f, "pressure Pa");
    near(result.second, 20.07f, 0.01f, "temperature C");
    require(driver.getDelay(BARO_MODE_PRESS) >= 10000, "OSR4096 needs conversion time");
  });
  test("second I2C address", []()
  {
    FakeBus bus;
    bus.address = 0x77;
    Device::Baro::BaroMS5611 driver;
    require(driver.begin(&bus), "0x77 detection failed");
    require(driver.getAddress() == 0x77, "wrong detected address");
  });
  test("SPI commands retain their command byte", []()
  {
    FakeBus bus;
    bus.spi = true;
    Device::Baro::BaroMS5611 driver;
    require(driver.begin(&bus, 5), "SPI detection failed");
    near(measure(driver).first, 100009.f, 1.f, "SPI pressure");
    require(bus.command == 0x48, "wrong pressure conversion command");
    require(!driver.begin(&bus), "SPI must require chip select");
  });
  test("real SPI command transport preserves legacy register transport", []()
  {
    TestSpi spi;
    Device::BusSPI bus(spi);
    uint8_t data[3]{};
    bus.read(5, 0, 3, data);
    require(spi.command == 0x80, "legacy SPI read bit changed");
    bus.readCommand(5, 0, 3, data);
    require(spi.command == 0, "ADC command was changed");
    bus.readCommand(5, 0xA0, 2, data);
    require(spi.command == 0xA0, "PROM command was changed");
    bus.write(5, 0xA0, 0, nullptr);
    require(spi.command == 0x20, "legacy SPI write mask changed");
    bus.writeCommand(5, 0xA0);
    require(spi.command == 0xA0, "bare command was changed");
  });
  test("CRC corruption and blank PROM", []()
  {
    FakeBus bus;
    Device::Baro::BaroMS5611 driver;
    bus.prom[1] ^= 1;
    require(!driver.begin(&bus), "bad CRC accepted");
    bus.prom.fill(0);
    require(!driver.begin(&bus), "blank PROM accepted");
    bus.prom.fill(0xFFFF);
    require(!driver.begin(&bus), "erased PROM accepted");
  });
  test("cold compensation uses first-order temperature", []()
  {
    FakeBus bus;
    Device::Baro::BaroMS5611 driver;
    require(driver.begin(&bus), "detection failed");
    struct Vector { uint32_t d2; int pressure; int temperature; };
    const Vector vectors[] = {
      {8500000, 99555, 1772}, {8000000, 95989, -62}, {7500000, 91910, -2130},
      {7000000, 85693, -4431}, {6500000, 76043, -6965}, {6000000, 62959, -9731},
      {9500000, 106051, 5149}
    };
    for (const auto& vector : vectors)
    {
      bus.d2 = vector.d2;
      const auto actual = measure(driver);
      near(actual.first, (float)vector.pressure, 0.f, "compensated pressure");
      near(actual.second, vector.temperature * 0.01f, 0.001f, "compensated temperature");
    }
  });
  test("ADC error invalidates sample instead of repeating old pressure", []()
  {
    FakeBus bus;
    Device::Baro::BaroMS5611 driver;
    require(driver.begin(&bus), "detection failed");
    measure(driver);
    bus.shortRead = true;
    const auto result = measure(driver);
    require(std::isnan(result.first) && std::isnan(result.second), "I2C failure reused old values");
    require(bus.errors >= 2, "bus errors were suppressed");
    bus.shortRead = false;
    near(measure(driver).first, 100009.f, 1.f, "recovery");
  });
  test("zero and saturated ADC are rejected", []()
  {
    FakeBus bus;
    Device::Baro::BaroMS5611 driver;
    require(driver.begin(&bus), "detection failed");
    for (uint32_t invalid : {0u, 0xFFFFFFu})
    {
      bus.d2 = invalid;
      require(std::isnan(measure(driver).first), "invalid temperature ADC accepted");
      bus.d2 = 8569150;
      bus.d1 = invalid;
      require(std::isnan(measure(driver).first), "invalid pressure ADC accepted");
    }
  });
  test("failed conversion command cannot produce valid ADC", []()
  {
    FakeBus bus;
    Device::Baro::BaroMS5611 driver;
    require(driver.begin(&bus), "detection failed");
    bus.failCommand = true;
    require(std::isnan(measure(driver).first), "failed command accepted");
    require(bus.errors == 2, "conversion failures not reported");
  });
  test("sea-level and elevated-site ground zero", []()
  {
    near(Utils::toAltitude(101325.f), 0.f, 0.001f, "sea-level height");
    Sensor::BaroAltitude estimator;
    uint32_t now = 20000;
    for (int i = 0; i <= 100; ++i)
    {
      estimator.update(pressureForHeight(1500.f), now, false);
      now += 20000;
    }
    require(estimator.calibrated, "elevated-site calibration failed");
    near(estimator.height, 0.f, 0.001f, "relative ground zero");
    near(estimator.absolute, 1500.f, 0.03f, "absolute elevation");
  });
  test("unstable calibration restarts", []()
  {
    Sensor::BaroAltitude estimator;
    uint32_t now = 20000;
    Sensor::BaroAltitude::Result result{};
    for (int i = 0; i <= 100; ++i)
    {
      result = estimator.update(101325.f + (i % 2 ? 80.f : -80.f), now, false);
      now += 20000;
    }
    require(result == Sensor::BaroAltitude::Result::RESTARTED, "noise window accepted");
    require(!estimator.calibrated, "noisy calibration finished");
    calibrate(estimator, now);
  });
  test("ground calibration cannot complete while armed", []()
  {
    Sensor::BaroAltitude estimator;
    for (uint32_t now = 20000; now < 4000000; now += 20000)
      estimator.update(101325.f, now, true);
    require(!estimator.calibrated, "calibrated in flight");
  });
  test("calibration needs a continuous sample window", []()
  {
    Sensor::BaroAltitude estimator;
    estimator.update(101325.f, 10000, false);
    estimator.update(101325.f, 3010000, false);
    require(!estimator.calibrated, "two isolated samples completed calibration");
    uint32_t now = 3030000;
    calibrate(estimator, now);
  });
  test("disarmed drift zero and armed ground reference freeze", []()
  {
    Sensor::BaroAltitude estimator;
    uint32_t now = 20000;
    calibrate(estimator, now);
    estimator.update(101200.f, now, false);
    near(estimator.height, 0.f, 0.001f, "thermal drift on ground");
    const float ground = estimator.groundAltitude;
    now += 20000;
    for (int i = 0; i < 200; ++i)
    {
      estimator.update(101100.f, now, true);
      now += 20000;
    }
    near(estimator.groundAltitude, ground, 0.001f, "ground reference moved in flight");
    require(estimator.height > 8.f, "in-flight pressure change zeroed out");
  });
  test("climb and descent vario use actual irregular intervals", []()
  {
    Sensor::BaroAltitude estimator;
    uint32_t now = 20000;
    calibrate(estimator, now);
    const uint32_t start = now;
    for (int i = 0; i < 250; ++i)
    {
      now += i % 2 ? 15000 : 35000;
      estimator.update(pressureForHeight((now - start) * 1e-6f), now, true);
    }
    near(estimator.vario, 1.f, 0.04f, "climb vario m/s");
    near(estimator.height, (now - start) * 1e-6f, 0.20f, "filtered climb height");
    const float top = (now - start) * 1e-6f;
    const uint32_t descentStart = now;
    for (int i = 0; i < 200; ++i)
    {
      now += i % 2 ? 15000 : 35000;
      estimator.update(pressureForHeight(top - (now - descentStart) * 1e-6f), now, true);
    }
    near(estimator.vario, -1.f, 0.04f, "descent vario m/s");
  });
  test("clock wrap during calibration and flight", []()
  {
    Sensor::BaroAltitude estimator;
    uint32_t now = std::numeric_limits<uint32_t>::max() - 1000000;
    calibrate(estimator, now);
    estimator.update(pressureForHeight(1.f), now, true);
    near(estimator.sampleDt, 0.02f, 0.00001f, "wrapped dt");
  });
  test("gap and first sample cannot create vario spikes", []()
  {
    Sensor::BaroAltitude estimator;
    uint32_t now = 20000;
    calibrate(estimator, now);
    now += 500000;
    estimator.update(pressureForHeight(10.f), now, true);
    near(estimator.vario, 0.f, 0.001f, "gap vario");
    near(estimator.height, 10.f, 0.03f, "gap height");
    require(estimator.sampleDt == 0.f, "gap must not be used as correction dt");
  });
  test("constant acceleration integrates once with correct kinematics", []()
  {
    Control::VerticalMotion estimator;
    for (int i = 0; i < 100; ++i) estimator.predict(2.f, 0.01f);
    near(estimator.height, 1.f, 0.0001f, "height after one second");
    near(estimator.velocity, 2.f, 0.0001f, "velocity after one second");
  });
  test("bias removal does not remove real climb acceleration", []()
  {
    Control::VerticalMotion estimator;
    estimator.bias = 0.2f;
    for (int i = 0; i < 100; ++i) estimator.predict(1.2f, 0.01f);
    near(estimator.height, 0.5f, 0.0001f, "bias-compensated height");
    near(estimator.bias, 0.2f, 0.0001f, "bias changed without baro residual");
  });
  test("fusion is independent of IMU loop rate", []()
  {
    const auto estimate = [](int rate)
    {
      Control::VerticalMotion motion;
      const float dt = 1.f / rate;
      for (int i = 0; i < 5 * rate; ++i)
      {
        motion.predict(0.f, dt);
        if ((i + 1) % (rate / 50) == 0)
          motion.correct(2.f, 0.f, 0.02f, 0.35f, 0.35f, 0.f);
      }
      return motion.height;
    };
    near(estimate(100), estimate(1000), 0.0001f, "rate-dependent fusion");
    require(estimate(100) > 1.6f && estimate(100) < 1.7f, "incorrect time-scaled correction");
  });
  test("takeoff waits below and at center, climbs above deadband", []()
  {
    Control::AltHoldStick stick;
    stick.begin(0.f, 0.f, true);
    for (float throttle : {-1.f, -0.2f, 0.f, 0.04f})
    {
      near(stick.update(throttle, 0.f, 0.05f, 1.f), 0.f, 0.f, "premature takeoff");
      require(stick.preparingTakeoff, "ground takeoff gate cleared");
    }
    require(stick.update(0.2f, 0.f, 0.05f, 1.f) > 0.f, "no climb above center");
    require(!stick.preparingTakeoff, "takeoff still gated");
  });
  test("center captures release height rather than integrated command", []()
  {
    Control::AltHoldStick stick;
    stick.begin(2.f, 0.f, false);
    stick.update(0.5f, 3.f, 0.05f, 1.f);
    stick.update(0.f, 3.2f, 0.05f, 1.f);
    near(stick.target, 3.2f, 0.0001f, "release target");
    stick.update(0.f, 3.5f, 0.05f, 1.f);
    near(stick.target, 3.2f, 0.0001f, "hold target moved");
  });
  test("full throttle ranges and asymmetric configured neutral", []()
  {
    Control::AltHoldStick stick;
    stick.begin(5.f, 0.f, false);
    near(stick.climbRate(1.f, 0.05f, 2.f), 2.f, 0.0001f, "max climb");
    near(stick.climbRate(-1.f, 0.05f, 2.f), -2.f, 0.0001f, "max descent");
    near(stick.climbRate(0.05f, 0.05f, 2.f), 0.f, 0.f, "positive deadband edge");
    near(stick.climbRate(-0.05f, 0.05f, 2.f), 0.f, 0.f, "negative deadband edge");
    stick.begin(5.f, 0.3f, false);
    near(stick.climbRate(1.f, 0.05f, 2.f), 2.f, 0.0001f, "asymmetric max climb");
    near(stick.climbRate(-1.f, 0.05f, 2.f), -2.f, 0.0001f, "asymmetric max descent");
  });
  test("rearm resets ground gate and velocity changes are bounded", []()
  {
    Control::AltHoldStick stick;
    stick.begin(0.f, 0.f, true);
    stick.update(1.f, 0.f, 0.05f, 1.f);
    stick.begin(0.f, 0.f, true);
    require(stick.preparingTakeoff && !stick.adjusting, "rearm retained flight state");
    near(Control::AltHoldStick::limitVelocity(10.f, 0.f, 0.02f), 0.0980665f, 0.00001f, "up acceleration");
    near(Control::AltHoldStick::limitVelocity(-10.f, 0.f, 0.02f), -0.1569064f, 0.00001f, "down acceleration");
  });
  test("position control uses linear/square-root braking and prevents one-step overshoot", []()
  {
    near(Control::AltHoldStick::holdVelocity(0.1f, 0.5f, 1.f, 0.02f), 0.05f, 0.0001f, "linear region");
    near(Control::AltHoldStick::holdVelocity(0.2f, 3.f, 1.f, 0.02f),
        std::sqrt(2.f * (0.2f - 1.f / 18.f)), 0.0001f, "square-root region");
    near(Control::AltHoldStick::holdVelocity(-0.2f, 3.f, 1.f, 0.02f),
        -std::sqrt(2.f * (0.2f - 1.f / 18.f)), 0.0001f, "negative square-root region");
    near(Control::AltHoldStick::holdVelocity(0.001f, 100.f, 1.f, 0.1f), 0.01f, 0.0001f, "one-step speed limit");
    near(Control::AltHoldStick::holdVelocity(100.f, 0.5f, 1.f, 0.02f), 1.f, 0.f, "max hold velocity");
  });
  test("INAV velocity gains have the correct cm/PWM to metre/throttle conversion", []()
  {
    near(100 * VEL_PTERM_SCALE, (100.f / 66.7f) * 100.f * 2.f / 1000.f, 0.000001f, "P gain conversion");
    near(50 * VEL_ITERM_SCALE, (50.f / 20.f) * 100.f * 2.f / 1000.f, 0.000001f, "I gain conversion");
    near(10 * VEL_DTERM_SCALE, (10.f / 100.f) * 100.f * 2.f / 1000.f, 0.000001f, "D gain conversion");
  });
  test("ground throttle stays at motor idle without integral accumulation", []()
  {
    auto pid = velocityPid();
    Control::AltHoldThrottle throttle;
    throttle.begin(pid, 0.f, -1.f, 0.f, true);
    for (int i = 0; i < 2000; ++i)
      near(throttle.update(pid, 0.f, 0.f, 0.f, -1.f, 0.002f, true), -1.f, 0.f, "ground throttle");
    near(pid.iTerm, -1.f, 0.f, "ground integral accumulated");
    const float first = throttle.update(pid, 0.01f, 0.f, 0.f, -1.f, 0.002f, false);
    require(first < -0.99f && first >= -1.f, "takeoff throttle jumped to hover");
  });
  test("airborne hold entry starts from previous throttle, not hover", []()
  {
    for (float previous : {-0.7f, 0.4f, 0.9f})
    {
      auto pid = velocityPid();
      Control::AltHoldThrottle throttle;
      throttle.begin(pid, 0.f, -1.f, 0.f, false, previous);
      const float first = throttle.update(pid, 0.f, 0.f, 0.f, -1.f, 0.002f, false);
      require(std::fabs(first - previous) < 0.05f, "entry jumped to hover");
      require(pid.iTerm >= pid.iLimitLow && pid.iTerm <= pid.iLimitHigh, "entry trim exceeds integral bounds");
    }
  });

  test("throttle anti-windup allows recovery from saturation", []()
  {
    auto pid = velocityPid();
    pid.Kp = 2.f;
    pid.Ki = 1.f;
    pid.Kd = pid.Kf = 0.f;
    pid.ptermFilter.begin();
    Control::AltHoldThrottle throttle;
    throttle.begin(pid, 0.f, -1.f, 0.f, false);
    for (int i = 0; i < 1000; ++i)
      throttle.update(pid, 1.f, 0.f, 0.f, -1.f, 0.002f, false);
    require(pid.iTerm < 0.01f, "integral wound up while saturated");
    const float previous = pid.iTerm;
    throttle.update(pid, -1.f, 0.f, 0.f, -1.f, 0.002f, false);
    require(pid.iTerm < previous, "integral cannot unwind");
  });
  test("closed-loop climb, hold and descent with production velocity PID", []()
  {
    auto pid = velocityPid();
    Control::AltHoldThrottle throttle;
    Control::AltHoldStick stick;
    Control::VerticalMotion motion;
    Sensor::BaroAltitude baro;
    uint32_t now = 20000;
    calibrate(baro, now);
    stick.begin(2.f, 0.f, false);
    throttle.begin(pid, 0.f, -1.f, 0.f, false);
    motion.height = 2.f;
    float actualHeight = 2.f;
    float actualVelocity = 0.f;
    float acceleration = 0.f;
    float desiredVelocity = 0.f;
    const float dt = 0.002f;
    const auto step = [&](float input, int steps)
    {
      for (int i = 0; i < steps; ++i)
      {
        now += 2000;
        motion.predict(acceleration, dt);
        if (i % 10 == 0)
        {
          baro.update(pressureForHeight(actualHeight), now, true);
          if (baro.sampleDt > 0.f)
            motion.correct(baro.height, baro.vario, baro.sampleDt, 0.35f, 0.35f, 0.01f);
        }
        float demand = stick.update(input, motion.height, 0.05f, 1.f);
        if (demand == 0.f)
          demand = Control::AltHoldStick::holdVelocity(stick.target - motion.height, 0.5f, 1.f, dt);
        desiredVelocity = Control::AltHoldStick::limitVelocity(demand, desiredVelocity, dt);
        const float output = throttle.update(pid, desiredVelocity, motion.velocity, 0.f, -1.f, dt, false);
        require(output >= -1.f && output <= 1.f, "throttle outside mixer range");
        acceleration = 9.80665f * output;
        actualHeight += actualVelocity * dt + 0.5f * acceleration * dt * dt;
        actualVelocity += acceleration * dt;
      }
    };
    step(1.f, 3000);
    require(actualHeight > 6.f, "commanded climb failed");
    near(actualVelocity, 1.f, 0.15f, "climb rate tracking");
    step(0.f, 1);
    const float target = stick.target;
    step(0.f, 5000);
    near(actualHeight, target, 0.25f, "hold altitude after ten seconds");
    near(actualVelocity, 0.f, 0.15f, "hold vario");
    step(-1.f, 2500);
    require(actualHeight < target - 3.f, "commanded descent failed");
    near(actualVelocity, -1.f, 0.15f, "descent rate tracking");
  });

  std::cout << count - failures << "/" << count << " tests passed\n";
  failures += runNavigationTests();
  return failures ? 1 : 0;
}
