#include "BaroMS5611.hpp"
#include <Arduino.h>
#include <limits>

// MS5611-01BA03 I2C addresses (CSB pin selects)
#define MS5611_ADDRESS_FIRST  0x76
#define MS5611_ADDRESS_SECOND 0x77

// Commands
#define MS5611_CMD_RESET      0x1E
#define MS5611_CMD_CONVERT_D1 0x48  // Convert pressure,    OSR=4096
#define MS5611_CMD_CONVERT_D2 0x58  // Convert temperature, OSR=4096
#define MS5611_CMD_ADC_READ   0x00
#define MS5611_CMD_PROM_BASE  0xA0  // PROM word i at (0xA0 + i*2)

// Some MS5611 clones are slower than datasheet typical timings on ESP32 I2C.
// 12ms matches the known-good standalone sketch behavior.
#define MS5611_CONVERSION_TIME_US 12000

namespace Xsfc::Device::Baro {

int BaroMS5611::begin(BusDevice* bus)
{
  if (bus->isSPI()) return 0; // SPI requires an explicit chip-select pin.
  return begin(bus, MS5611_ADDRESS_FIRST) ? 1 : begin(bus, MS5611_ADDRESS_SECOND) ? 1 : 0;
}

int BaroMS5611::begin(BusDevice* bus, uint8_t addr)
{
  setBus(bus, addr);
  _temperatureValid = false;
  _conversionStarted = false;
  if (!sendCommand(MS5611_CMD_RESET)) return 0;
  delay(10); // datasheet min 2.8ms; use 10ms for reliable PROM read on all chips

  if (!testConnection()) return 0;

  return 1;
}

BaroDeviceType BaroMS5611::getType() const
{
  return BARO_MS5611;
}

float BaroMS5611::readTemperature()
{
  const uint32_t D2 = _conversionStarted ? readADC() : 0;
  _conversionStarted = false;
  _temperatureValid = D2 != 0 && D2 != 0xFFFFFF;
  if (!_temperatureValid) return std::numeric_limits<float>::quiet_NaN();

  _dT = (int64_t)D2 - ((int64_t)_c[4] << 8);
  _temperatureFirstOrder = 2000 + ((_dT * _c[5]) >> 23);
  const int64_t T2 = _temperatureFirstOrder < 2000 ? (_dT * _dT) >> 31 : 0;
  return (_temperatureFirstOrder - T2) * 0.01f;
}

float BaroMS5611::readPressure()
{
  const uint32_t D1 = _conversionStarted ? readADC() : 0;
  _conversionStarted = false;
  if (!_temperatureValid || D1 == 0 || D1 == 0xFFFFFF)
    return std::numeric_limits<float>::quiet_NaN();

  const int64_t TEMP = _temperatureFirstOrder;

  int64_t OFF  = ((int64_t)_c[1] << 16) + (((int64_t)_c[3] * _dT) >> 7);
  int64_t SENS = ((int64_t)_c[0] << 15) + (((int64_t)_c[2] * _dT) >> 8);

  if (TEMP < 2000)
  {
    int64_t tmp   = (TEMP - 2000) * (TEMP - 2000);
    int64_t OFF2  = (5LL * tmp) >> 1;
    int64_t SENS2 = (5LL * tmp) >> 2;
    if (TEMP < -1500)
    {
      int64_t tmp2 = (TEMP + 1500) * (TEMP + 1500);
      OFF2  += 7LL * tmp2;
      SENS2 += (11LL * tmp2) >> 1;
    }
    OFF  -= OFF2;
    SENS -= SENS2;
  }

  int64_t P = ((int64_t)D1 * SENS >> 21) - OFF;
  P >>= 15;

  return (float)P;
}

void BaroMS5611::setMode(BaroDeviceMode mode)
{
  if (mode == BARO_MODE_TEMP)
    _conversionStarted = sendCommand(MS5611_CMD_CONVERT_D2);
  else
    _conversionStarted = sendCommand(MS5611_CMD_CONVERT_D1);
}

int BaroMS5611::getDelay(BaroDeviceMode mode) const
{
  (void)mode;
  return MS5611_CONVERSION_TIME_US;
}

bool BaroMS5611::testConnection()
{
  return readPROM();
}

bool BaroMS5611::sendCommand(uint8_t cmd)
{
  return _bus->writeCommand(_addr, cmd);
}

// ADC read: send 0x00 then read 3 bytes.
uint32_t BaroMS5611::readADC()
{
  uint8_t buf[3] = {0, 0, 0};
  if (_bus->readCommand(_addr, MS5611_CMD_ADC_READ, 3, buf) != 3) return 0;
  return ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
}

bool BaroMS5611::readPROM()
{
  uint16_t prom[8];
  for (int i = 0; i < 8; i++)
  {
    uint8_t buf[2] = {0, 0};
    if (_bus->readCommand(_addr, MS5611_CMD_PROM_BASE + i * 2, 2, buf) != 2) return false;
    prom[i] = ((uint16_t)buf[0] << 8) | buf[1];
  }

  bool anyCoef = false;
  for (int i = 1; i <= 6; i++)
  {
    if (prom[i] != 0 && prom[i] != 0xFFFF)
    {
      anyCoef = true;
      break;
    }
  }
  if (!anyCoef) return false;

  uint8_t crcExpected = prom[7] & 0x000F;
  if (crc4(prom) != crcExpected) return false;

  for (int i = 0; i < 6; i++)
    _c[i] = prom[i + 1];

  return true;
}

uint8_t BaroMS5611::crc4(uint16_t prom[8])
{
  uint16_t n_rem  = 0;
  uint16_t saved  = prom[7];
  prom[7]        &= 0xFF00;

  for (int cnt = 0; cnt < 16; cnt++)
  {
    if (cnt % 2 == 1) n_rem ^= (prom[cnt >> 1] & 0x00FF);
    else              n_rem ^= (prom[cnt >> 1] >> 8);
    for (int n_bit = 8; n_bit > 0; n_bit--)
    {
      if (n_rem & 0x8000) n_rem = (n_rem << 1) ^ 0x3000;
      else                n_rem = n_rem << 1;
    }
  }
  prom[7] = saved;
  return (n_rem >> 12) & 0x000F;
}

} // namespace Xsfc::Device::Baro
