#pragma once

#include "FlightModes.hpp"

namespace Xsfc::Control {

struct HoldSafety
{
  static bool needsReset(bool engaged, bool previousRangeSource, bool rangeSource)
  {
    // RANGEHOLD and POSHOLD share the same vertical controller and target.
    return !engaged || previousRangeSource != rangeSource;
  }

  bool update(bool armed, uint32_t activeModes, uint32_t requestedModes, bool baroReady,
      bool rangeReady, bool baroFallback, bool rangeWasEngaged)
  {
    // Only an accepted mode may select the control source. A rejected range
    // request must not disable an otherwise active barometric hold.
    const bool rangeRequested = requestedModes & RANGE_HOLD_MODES;
    rangeSource = activeModes & RANGE_HOLD_MODES;
    if (!rangeRequested || !armed) rangeLost = false;
    if (armed && rangeSource && rangeWasEngaged && !rangeReady) rangeLost = true;
    if (!armed) return false;
    if (rangeSource) return (activeModes & RANGE_HOLD_MODES) && rangeReady && !rangeLost;
    return (activeModes & BARO_HOLD_MODES) && (baroReady || !baroFallback);
  }

  bool rangeSource = false;
  bool rangeLost = false;
};

} // namespace Xsfc::Control
