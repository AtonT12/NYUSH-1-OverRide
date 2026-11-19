#pragma once

#include <stdint.h>

namespace control {

// Limits and rates for one mechanism axis, expressed in degrees relative to a
// homed zero. All of these are placeholders until the mechanical team measures
// the real travel; see docs/hardware.md.
struct AxisCal {
  double minDeg;
  double maxDeg;
  double toleranceDeg;     // "at target" window
  double approachDeg;      // start slowing down this close to the target
  double maxDegPerSec;     // rate limit for closed-loop moves
  double homingCurrentAmp; // stall current that marks the hard stop
  uint16_t homingStableTicks;
  uint32_t homingTimeoutMs;
};

// Clamps a requested target into the soft limits. Sets *clamped when it had to
// intervene so the caller can report "limit reached" instead of silently
// pretending the move was accepted.
double clampToLimits(const AxisCal& cal, double targetDeg, bool* clamped);

// Moves fromDeg towards toDeg by at most maxStepDeg.
double limitRate(double fromDeg, double toDeg, double maxStepDeg);

// True when the axis is within bandDeg of its target.
bool inBand(double actualDeg, double targetDeg, double bandDeg);

// Speed scale to apply inside the approach window: 0.25 when close, 1.0
// otherwise. Slamming a lift into its soft limit at full speed is how gearboxes
// get stripped.
double approachSpeedScale(const AxisCal& cal, double remainingDeg);

// Homing is done by driving into the mechanical stop and watching for a stall.
// Requiring both a current spike and a lack of position change rejects the
// false positive you get when the arm is simply heavy at the start of the move.
bool shouldStopHoming(double currentAmp, double posDeltaDeg, uint16_t stableTicks,
                      const AxisCal& cal);

}  // namespace control
