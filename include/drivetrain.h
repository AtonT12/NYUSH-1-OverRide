#pragma once

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

// X-drive mixing for deadbanded controller axes in the -100..100 range.
void driveX(double forwardAxis, double strafeAxis, double turnAxis);

// Stops all bound drivetrain motors with brake mode.
void stop();

// Resets every drivetrain motor encoder to zero. Only valid when the robot is
// at rest and the full eight-motor drivetrain is bound.
void resetEncoders();

}  // namespace Drivetrain
