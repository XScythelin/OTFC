#pragma once

#include <cstdint>
#include <cstddef>

class Print
{
public:
  virtual ~Print() = default;
  virtual size_t write(uint8_t) = 0;
  virtual size_t write(const uint8_t* buffer, size_t length)
  {
    size_t written = 0;
    while (written < length && write(buffer[written])) ++written;
    return written;
  }
};
class Stream : public Print {};
inline void delay(unsigned long) {}
inline uint32_t millis() { static uint32_t now = 0; return ++now; }
inline uint32_t testMicros = 0;
inline uint32_t micros() { return testMicros; }

constexpr int MSBFIRST = 1;
constexpr int SPI_MODE0 = 0;
struct SPISettings
{
  SPISettings(uint32_t, int, int) {}
};

struct TestSpi
{
  void beginTransaction(SPISettings) {}
  void endTransaction() {}
  uint8_t transfer(uint8_t value) { command = value; return 0; }
  void transferBytes(const uint8_t*, uint8_t* output, uint8_t length)
  {
    if (output)
      for (uint8_t i = 0; i < length; ++i) output[i] = 0;
  }
  uint8_t command = 0;
};
