#pragma once

#include <cstddef>
#include <cstdint>

namespace Xsfc {

enum FlightMode {
  MODE_ARMED,
  MODE_AIRMODE,
  MODE_ANGLE,
  MODE_ALTHOLD,
  MODE_BUZZER,
  MODE_FAILSAFE,
  MODE_BLACKBOX,
  MODE_BLACKBOX_ERASE,
  MODE_GPIO_OUTPUT = 9, // Preserve the saved IDs; ID 8 was the old MODE_COUNT.
  MODE_MAG,
  // IDs 11 (SURFACE), 12 (horizontal-only POSHOLD) are retired.
  MODE_RANGEHOLD = 13,
  MODE_POSHOLD = 14, // Former FLOWHOLD: full flow + range + angle hold.
  MODE_COUNT = 15,
};

struct FlightModeInfo
{
  FlightMode id;
  const char* name;
};

constexpr FlightModeInfo FLIGHT_MODES[] = {
  {MODE_ARMED, "ARM"}, {MODE_AIRMODE, "AIRMODE"}, {MODE_ANGLE, "ANGLE"},
  {MODE_ALTHOLD, "ALTHOLD"}, {MODE_BUZZER, "BEEPER"}, {MODE_FAILSAFE, "FAILSAFE"},
  {MODE_BLACKBOX, "BLACKBOX"}, {MODE_BLACKBOX_ERASE, "BLACKBOXERASE"},
  {MODE_GPIO_OUTPUT, "GPIO"}, {MODE_MAG, "MAG"},
  {MODE_POSHOLD, "POSHOLD"}, {MODE_RANGEHOLD, "RANGEHOLD"}
};

constexpr uint32_t modeBit(FlightMode mode) { return 1u << mode; }
constexpr bool isKnownFlightMode(uint8_t mode) { return mode < MODE_COUNT && mode != 8 && mode != 11 && mode != 12; }
constexpr uint32_t RANGE_HOLD_MODES = modeBit(MODE_RANGEHOLD) | modeBit(MODE_POSHOLD);
constexpr uint32_t BARO_HOLD_MODES = modeBit(MODE_ALTHOLD);
constexpr uint32_t POSITION_HOLD_MODES = modeBit(MODE_POSHOLD);

inline uint32_t mspModeMask(uint32_t mask)
{
  uint32_t result = 0;
  for (size_t i = 0; i < sizeof(FLIGHT_MODES) / sizeof(FLIGHT_MODES[0]); ++i)
    if (mask & modeBit(FLIGHT_MODES[i].id)) result |= 1u << i;
  return result;
}

static_assert(MODE_COUNT <= 32, "Mode flags must fit in 32 bits");

} // namespace Xsfc
