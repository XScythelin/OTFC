#pragma once

#include <cstdint>

namespace Xsfc {

enum { GYRO_DLPF_256 = 0, GYRO_DLPF_EX = 7, GYRO_DLPF_OFF = 8 };

// The rate converter only consumes these fields; board configuration is not needed by native tests.
struct InputConfig
{
  uint8_t expo[3] = {0, 0, 0};
  uint8_t rate[3] = {20, 20, 30};
  uint8_t superRate[3] = {40, 40, 36};
  int16_t rateLimit[3] = {1998, 1998, 1998};
  int8_t rateType = 3;
};

}
