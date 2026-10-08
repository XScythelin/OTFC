#pragma once

#include <cstdint>

namespace Xsfc {

enum SensorAlign {
  ALIGN_DEFAULT = 0, ALIGN_CW0_DEG = 1, ALIGN_CW90_DEG = 2, ALIGN_CW180_DEG = 3,
  ALIGN_CW270_DEG = 4, ALIGN_CW0_DEG_FLIP = 5, ALIGN_CW90_DEG_FLIP = 6,
  ALIGN_CW180_DEG_FLIP = 7, ALIGN_CW270_DEG_FLIP = 8, ALIGN_CUSTOM = 9
};

inline void alignSensorVector(float& x, float& y, float& z, uint8_t alignment)
{
  const float oldX = x;
  const float oldY = y;
  switch (alignment)
  {
    case ALIGN_CW90_DEG: x = oldY; y = -oldX; break;
    case ALIGN_CW180_DEG: x = -oldX; y = -oldY; break;
    case ALIGN_CW270_DEG: x = -oldY; y = oldX; break;
    case ALIGN_CW0_DEG_FLIP: x = -oldX; z = -z; break;
    case ALIGN_CW90_DEG_FLIP: x = oldY; y = oldX; z = -z; break;
    case ALIGN_CW180_DEG_FLIP: y = -oldY; z = -z; break;
    case ALIGN_CW270_DEG_FLIP: x = -oldY; y = -oldX; z = -z; break;
    default: break;
  }
}

} // namespace Xsfc
