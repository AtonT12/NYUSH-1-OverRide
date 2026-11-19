#pragma once

#include "control/command.h"

namespace control {

// Calibration for the input shaping stage. Values live in normalized units so
// they survive a change of the control-loop period: slew rates are per second,
// not per frame. A slew rate of 0 disables the limiter on that channel.
struct ShapingCal {
  double translationDeadband;    // 0.0; the drive axes keep full resolution
  double turnDeadband;           // 0.20
  double liftCoarseDeadband;     // 0.20
  double translationSlewPerSec;  // 10.0
  double turnSlewPerSec;         // 5.0
  double liftCoarseSlewPerSec;   // 0.0, unshaped as in the reference
};

// Held between calls so the slew limiter has memory.
struct SlewState {
  double forward;
  double strafe;
  double turn;
  double liftCoarse;

  constexpr SlewState() : forward(0.0), strafe(0.0), turn(0.0), liftCoarse(0.0) {}
};

// Radial deadband with linear rescaling. Input outside [-1, 1] is clamped
// first; the deadband is clamped to [0, 0.99] so the divisor never collapses.
double applyDeadbandUnit(double value, double deadband);

// Same shaping for callers that speak controller units (-100..100). Rounds
// instead of truncating: truncation towards zero biases the two halves of the
// stick by up to one unit each.
int applyDeadbandPercent(int value, int deadband);

// Moves current towards target by at most maxDelta.
double slewScalar(double current, double target, double maxDelta);

// Same, but along the 2D vector (x, y): the step is limited in magnitude and
// the direction is preserved. Limiting each axis separately would bend a
// diagonal push towards the axis with more headroom.
void slewVector(double* x, double* y, double targetX, double targetY, double maxDelta);

// Deadband + slew. dtSec <= 0 keeps the previous slew state (frozen output).
UserCommand shape(const UserCommand& raw, SlewState* state, const ShapingCal& cal,
                  double dtSec);

}  // namespace control
