#include "robot-config.h"

#include "motor-control.h"

// Brain and Controller are confirmed hardware and do not need invented ports.
vex::brain Brain;
vex::controller Controller;

namespace RobotHardware {

MotorPair driveFrontLeft = {nullptr, nullptr};
MotorPair driveFrontRight = {nullptr, nullptr};
MotorPair driveRearLeft = {nullptr, nullptr};
MotorPair driveRearRight = {nullptr, nullptr};
MotorPair primaryLift = {nullptr, nullptr};

vex::motor* toggleMotor = nullptr;
vex::motor* secondaryLiftMotor = nullptr;
vex::motor* clawMotor = nullptr;

namespace {

bool pairReady(const MotorPair& pair) {
  return pair.motorA != nullptr && pair.motorB != nullptr;
}

void stopPair(const MotorPair& pair, vex::brakeType mode) {
  MotorControl::stopMotor(pair.motorA, mode);
  MotorControl::stopMotor(pair.motorB, mode);
}

}  // namespace

bool drivetrainReady() {
  return pairReady(driveFrontLeft) && pairReady(driveFrontRight) &&
         pairReady(driveRearLeft) && pairReady(driveRearRight);
}

bool toggleReady() {
  return toggleMotor != nullptr;
}

bool primaryLiftReady() {
  return pairReady(primaryLift);
}

bool secondaryLiftReady() {
  return secondaryLiftMotor != nullptr;
}

bool clawReady() {
  return clawMotor != nullptr;
}

bool allReady() {
  return drivetrainReady() && toggleReady() && primaryLiftReady() &&
         secondaryLiftReady() && clawReady();
}

void stopAllMotors(vex::brakeType mode) {
  stopPair(driveFrontLeft, mode);
  stopPair(driveFrontRight, mode);
  stopPair(driveRearLeft, mode);
  stopPair(driveRearRight, mode);
  stopPair(primaryLift, mode);

  MotorControl::stopMotor(toggleMotor, mode);
  MotorControl::stopMotor(secondaryLiftMotor, mode);
  MotorControl::stopMotor(clawMotor, mode);
}

}  // namespace RobotHardware
