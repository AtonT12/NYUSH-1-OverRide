#pragma once

#include "control/command.h"

namespace Drivetrain {

bool isReady();

// Low-level wheel-group command in RPM. Motor directions are normalized per
// group so positive means "drive this wheel forward."
void setWheelRpms(double frontLeft, double frontRight, double rearLeft,
                  double rearRight);

// Single entry point for a normalized command: mixes, proportionally clamps to
// the active mode's wheel cap, and issues the four wheel-group commands.
void apply(const control::UserCommand& command);

// Stops all bound drivetrain motors with brake mode.
void stop();

// Resets every drivetrain motor encoder to zero. Only valid when the robot is
// at rest and the full eight-motor drivetrain is bound.
void resetEncoders();

}  // namespace Drivetrain
