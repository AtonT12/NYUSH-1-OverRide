#pragma once

namespace Drivetrain {

enum class Wheel {
  FrontLeft,
  FrontRight,
  RearLeft,
  RearRight
};

bool isReady();

// Low-level wheel-group commands. Motor directions are normalized per group so
// positive means "drive this wheel forward." These functions still do not
// contain an X-drive mixing formula and must not be connected directly to an
// operator joystick until the full wheel-axis sign convention is verified.
void setWheelPower(Wheel wheel, double percent);
void setWheelPowers(double frontLeft, double frontRight, double rearLeft,
                    double rearRight);

// Stops all bound drivetrain motors with brake mode.
void stop();

// Resets every drivetrain motor encoder to zero. Only valid when the robot is
// at rest and the full eight-motor drivetrain is bound.
void resetEncoders();

}  // namespace Drivetrain
