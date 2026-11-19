#pragma once

#include "control/command.h"

namespace control {

struct WheelRpm {
  double fl;
  double fr;
  double rl;
  double rr;

  constexpr WheelRpm() : fl(0.0), fr(0.0), rl(0.0), rr(0.0) {}
};

struct DriveCal {
  double translationRpm;  // wheel RPM at full stick translation
  double turnRpm;         // wheel RPM at full stick rotation
  double wheelCapRpm;     // hard ceiling applied after mixing
};

// Four-wheel omni X-drive mixing.
//
//   fl = (strafe + forward) / sqrt(2) * T + turn * R
//   fr = (-strafe + forward) / sqrt(2) * T - turn * R
//   rl = (-strafe + forward) / sqrt(2) * T + turn * R
//   rr = (strafe + forward) / sqrt(2) * T - turn * R
//
// The 1/sqrt(2) is what makes a pure forward push (forward = 1, strafe = 0)
// come out at 0.707 * T on every wheel, which is the correct ratio for a
// 45-degree omni layout.
WheelRpm mixXDrive(const UserCommand& command, const DriveCal& cal);

// Scales all four wheels by the same factor so the largest magnitude lands on
// capRpm. Per-wheel clipping instead would distort the ratio between wheels
// and therefore bend the robot's actual heading away from the commanded one.
WheelRpm clampWheels(WheelRpm wheels, double capRpm);

// Largest absolute wheel magnitude, useful for diagnostics and tests.
double wheelPeak(const WheelRpm& wheels);

}  // namespace control
