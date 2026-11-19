#include "control/timebase.h"

namespace control {
namespace {

const uint16_t kFallbackPeriodMs = 20;

}  // namespace

void tickReset(TickGrid* grid, uint32_t nowMs, uint16_t periodMs) {
  if (grid == 0) {
    return;
  }
  grid->startMs = nowMs;
  grid->periodMs = periodMs == 0 ? kFallbackPeriodMs : periodMs;
  grid->index = 0;
}

uint32_t tickTime(const TickGrid& grid) {
  return grid.startMs + grid.index * static_cast<uint32_t>(grid.periodMs);
}

bool tickDue(const TickGrid& grid, uint32_t nowMs) {
  // Signed difference: correct even when the 32-bit millisecond counter wraps.
  return static_cast<int32_t>(nowMs - tickTime(grid)) >= 0;
}

uint32_t tickAdvance(TickGrid* grid, uint32_t nowMs) {
  if (grid == 0 || grid->periodMs == 0) {
    return 0;
  }
  if (!tickDue(*grid, nowMs)) {
    return 0;
  }
  const uint32_t lateMs = nowMs - tickTime(*grid);
  const uint32_t steps = lateMs / grid->periodMs + 1;
  grid->index += steps;
  return steps;
}

uint32_t tickElapsed(const TickGrid& grid, uint32_t nowMs) {
  return nowMs - grid.startMs;
}

}  // namespace control
