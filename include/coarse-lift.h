#pragma once

// Coarse lift soft limit on port 5, ported from NYU-Vex-2026 src/main.rs:302-331.
//
// There is no limit switch. The lowest negative angle the motor has reported is
// treated as the bottom of travel; the top is capped at 202 degrees above it,
// and the last 15 degrees are covered by a slow position target. Ported as-is,
// including its precondition: the reference only works if the lift starts at or
// below its bottom stop, otherwise the cap is measured from power-on position.
namespace CoarseLift {

// Drops the homing reference. Call on entry to a driver or autonomous period.
void reset();

// One control tick. Positive `amount` raises. `maxRpm` is the unshaped speed.
void apply(double amount, double maxRpm);

}  // namespace CoarseLift
