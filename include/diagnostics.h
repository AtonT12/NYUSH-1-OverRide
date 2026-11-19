#pragma once

// Bench-only tool. Steps one motor at a time so wheel and mechanism directions
// can be signed off against docs/hardware.md without touching main.cpp.
namespace Diagnostics {

// Blocking. Returns after the operator presses Y, having stopped every motor.
// Entered from pre_auton only when X and B are held at power-on, so it cannot
// be reached by accident during a match.
void run();

}  // namespace Diagnostics
