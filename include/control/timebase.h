#pragma once

#include <stdint.h>

namespace control {

// Absolute schedule for a fixed-rate control loop.
//
// The naive `wait(period)` at the end of every iteration accumulates the work
// done inside the loop: a 20 ms period becomes 22 ms, and a 30 s autonomous
// routine finishes seconds late. Indexing a grid against a fixed start time
// decouples the schedule from the work, which is the whole point.
struct TickGrid {
  uint32_t startMs;
  uint16_t periodMs;
  uint32_t index;

  constexpr TickGrid() : startMs(0), periodMs(20), index(0) {}
};

void tickReset(TickGrid* grid, uint32_t nowMs, uint16_t periodMs);

// Ideal, jitter-free time of the current tick.
uint32_t tickTime(const TickGrid& grid);

// True when the current tick's deadline has passed. Unsigned subtraction keeps
// this correct across the 32-bit millisecond counter wrap.
bool tickDue(const TickGrid& grid, uint32_t nowMs);

// Consumes every tick whose deadline has passed and returns how many were
// consumed (0 when not due). Skipping instead of catching up keeps playback
// monotonic: a late loop must not make the routine run fast to compensate.
uint32_t tickAdvance(TickGrid* grid, uint32_t nowMs);

// Milliseconds since the grid started at the given time.
uint32_t tickElapsed(const TickGrid& grid, uint32_t nowMs);

}  // namespace control
