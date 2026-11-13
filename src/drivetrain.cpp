#include "drivetrain.h"

#include "constants.h"
#include "motor-control.h"
#include "robot-config.h"

namespace Drivetrain {

namespace {

void applyPair(const RobotHardware::MotorPair& pair, double percent) {
  const double limited =
      MotorControl::limitMagnitude(percent, RobotConstants::kDriveMaxPercent);
  MotorControl::spinPercent(pair.motorA, limited);
  MotorControl::spinPercent(pair.motorB, limited);
}

void stopPair(const RobotHardware::MotorPair& pair) {
  MotorControl::stopMotor(pair.motorA, vex::brakeType::brake);
  MotorControl::stopMotor(pair.motorB, vex::brakeType::brake);
}

}  // namespace

bool isReady() {
  return RobotHardware::drivetrainReady();
}

void setWheelPower(Wheel wheel, double percent) {
  if (!isReady()) {
    return;
  }

  switch (wheel) {
    case Wheel::FrontLeft:
      applyPair(RobotHardware::driveFrontLeft, percent);
      break;
    case Wheel::FrontRight:
      applyPair(RobotHardware::driveFrontRight, percent);
      break;
    case Wheel::RearLeft:
      applyPair(RobotHardware::driveRearLeft, percent);
      break;
    case Wheel::RearRight:
      applyPair(RobotHardware::driveRearRight, percent);
      break;
  }
}

void setWheelPowers(double frontLeft, double frontRight, double rearLeft,
                    double rearRight) {
  if (!isReady()) {
    return;
  }

  setWheelPower(Wheel::FrontLeft, frontLeft);
  setWheelPower(Wheel::FrontRight, frontRight);
  setWheelPower(Wheel::RearLeft, rearLeft);
  setWheelPower(Wheel::RearRight, rearRight);
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
