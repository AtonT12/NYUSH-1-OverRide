#pragma once

#include "control/command.h"

namespace Drivetrain {

enum class Wheel {
  FrontLeft,
  FrontRight,
  RearLeft,
  RearRight
};

bool isReady();

// Low-level wheel-group commands in RPM. Motor directions are normalized per
// group so positive means "drive this wheel forward."
void setWheelRpm(Wheel wheel, double rpm);
void setWheelRpms(double frontLeft, double frontRight, double rearLeft,
                  double rearRight);

// Single entry point for a normalized command: mixes, proportionally clamps to
// the active mode's wheel cap, and issues the four wheel-group commands.
void apply(const control::UserCommand& command);

// X-drive mixing for deadbanded controller axes in the -100..100 range.
// Kept for callers that still speak controller units; it converts and defers to
// apply() so there is only one implementation of the mixing.
void driveX(double forwardAxis, double strafeAxis, double turnAxis);

// Stops all bound drivetrain motors with brake mode.
void stop();

// Resets every drivetrain motor encoder to zero. Only valid when the robot is
// at rest and the full eight-motor drivetrain is bound.
void resetEncoders();

}  // namespace Drivetrain
