#pragma once

#include <cmath>

namespace Xsfc::Connect {

inline float altitudeReadout(bool armed, bool barometerAvailable, float pressureAltitude, float holdHeight)
{
  return !armed && barometerAvailable && std::isfinite(pressureAltitude) ? pressureAltitude : holdHeight;
}

}
