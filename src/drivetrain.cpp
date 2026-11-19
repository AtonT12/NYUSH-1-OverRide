#include "drivetrain.h"

#include "constants.h"
#include "control/kinematics.h"
#include "motor-control.h"
#include "robot-config.h"

namespace Drivetrain {

namespace {

control::DriveCal driveCal() {
  control::DriveCal cal;
  cal.translationRpm = RobotConstants::kDriveTranslationRpm;
  cal.turnRpm = RobotConstants::kDriveTurnRpm;
  // Bench mode exists so a direction check can be read by eye. Capping here
  // rather than in the caller keeps every consumer of driveX (teleop, playback,
  // diagnostics) inside the same limit.
  cal.wheelCapRpm = RobotHardware::gRunMode == RobotConstants::RunMode::Bench
                        ? RobotConstants::kBenchWheelCapRpm
                        : RobotConstants::kDriveMaxCommandRpm;
  return cal;
}

void applyPair(const RobotHardware::MotorPair& pair, double rpm) {
  const double limited = MotorControl::limitMagnitude(
      rpm, RobotConstants::kDriveMaxCommandRpm);
  MotorControl::spinRpm(pair.motorA, limited);
  MotorControl::spinRpm(pair.motorB, limited);
}

void stopPair(const RobotHardware::MotorPair& pair) {
  MotorControl::stopMotor(pair.motorA, vex::brakeType::brake);
  MotorControl::stopMotor(pair.motorB, vex::brakeType::brake);
}

}  // namespace

bool isReady() {
  return RobotHardware::drivetrainReady();
}

void setWheelRpm(Wheel wheel, double rpm) {
  if (!isReady()) {
    // Returning alone would stop new commands but not the ones already issued,
    // so the remaining groups would keep their last RPM. A missing motor can
    // never be compensated by the other groups, so stop the whole drivetrain.
    stop();
    return;
  }

  switch (wheel) {
    case Wheel::FrontLeft:
      applyPair(RobotHardware::driveFrontLeft, rpm);
      break;
    case Wheel::FrontRight:
      applyPair(RobotHardware::driveFrontRight, rpm);
      break;
    case Wheel::RearLeft:
      applyPair(RobotHardware::driveRearLeft, rpm);
      break;
    case Wheel::RearRight:
      applyPair(RobotHardware::driveRearRight, rpm);
      break;
  }
}

void setWheelRpms(double frontLeft, double frontRight, double rearLeft,
                  double rearRight) {
  if (!isReady()) {
    stop();
    return;
  }

  // Drive the pairs directly: routing through setWheelRpm would re-run the
  // readiness probe (eight Smart Port queries) four more times per loop.
  applyPair(RobotHardware::driveFrontLeft, frontLeft);
  applyPair(RobotHardware::driveFrontRight, frontRight);
  applyPair(RobotHardware::driveRearLeft, rearLeft);
  applyPair(RobotHardware::driveRearRight, rearRight);
}

// Axis conventions (V5 controller, official mapping):
//   Axis3 = left stick vertical,  +100 pushed up
//   Axis4 = left stick horizontal, +100 pushed right
//   Axis1 = right stick horizontal, +100 pushed right
// Positive wheel RPM means "drive forward" after the per-motor normalization in
// robot-config.cpp, so the axes map to the mixer without a sign change.
//
// If the robot later drives opposite to the sticks, fix the mixer signs here or
// the per-motor reverse flags in robot-config.cpp, never both: an axis inversion
// plus a motor inversion cancels out and looks correct on the bench.
void apply(const control::UserCommand& command) {
  const control::DriveCal cal = driveCal();
  const control::WheelRpm wheels =
      control::clampWheels(control::mixXDrive(command, cal), cal.wheelCapRpm);
  setWheelRpms(wheels.fl, wheels.fr, wheels.rl, wheels.rr);
}

void driveX(double forwardAxis, double strafeAxis, double turnAxis) {
  // Mixing and the proportional clamp live in the host-testable control core so
  // the same maths can be verified without a robot. This function is only the
  // adapter: controller units -> normalized command -> wheel RPM.
  control::UserCommand command;
  command.forward = forwardAxis / 100.0;
  command.strafe = strafeAxis / 100.0;
  command.turn = turnAxis / 100.0;
  apply(command);
}

void stop() {
  stopPair(RobotHardware::driveFrontLeft);
  stopPair(RobotHardware::driveFrontRight);
  stopPair(RobotHardware::driveRearLeft);
  stopPair(RobotHardware::driveRearRight);
}

void resetEncoders() {
  if (!isReady()) {
    return;
  }

  RobotHardware::driveFrontLeft.motorA->setPosition(0.0, vex::degrees);
  RobotHardware::driveFrontLeft.motorB->setPosition(0.0, vex::degrees);
  RobotHardware::driveFrontRight.motorA->setPosition(0.0, vex::degrees);
  RobotHardware::driveFrontRight.motorB->setPosition(0.0, vex::degrees);
  RobotHardware::driveRearLeft.motorA->setPosition(0.0, vex::degrees);
  RobotHardware::driveRearLeft.motorB->setPosition(0.0, vex::degrees);
  RobotHardware::driveRearRight.motorA->setPosition(0.0, vex::degrees);
  RobotHardware::driveRearRight.motorB->setPosition(0.0, vex::degrees);
}

}  // namespace Drivetrain
