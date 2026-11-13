#include "robot-config.h"

#include "motor-control.h"

// Brain and Controller are confirmed hardware.
vex::brain Brain;
vex::controller Controller;

namespace {

// Confirmed port and direction map.
//
// Robot convention: the Claw is the front, the Toggle is the rear. Each drive
// group is normalized so a positive module command drives that wheel forward.
// Native-positive directions were supplied by the hardware owner; the reverse
// flag is set independently for every motor where that direction is opposite.
//
// Cartridges are not encoded here because they are still unknown. All current
// module commands use percent power, not RPM or physical distance conversions.

// FL native positive = backward -> invert to make positive = forward.
vex::motor driveFrontLeftAMotor(vex::PORT1, true);
vex::motor driveFrontLeftBMotor(vex::PORT2, true);

// FR native positive = forward.
vex::motor driveFrontRightAMotor(vex::PORT4, false);
vex::motor driveFrontRightBMotor(vex::PORT14, false);

// RL native positive = backward -> invert to make positive = forward.
vex::motor driveRearLeftAMotor(vex::PORT7, true);
vex::motor driveRearLeftBMotor(vex::PORT8, true);

// RR native positive = forward.
vex::motor driveRearRightAMotor(vex::PORT3, false);
vex::motor driveRearRightBMotor(vex::PORT19, false);

// Lift A native positive = raise. Lift B native positive = lower, so invert B
// to make positive command = raise for the complete subsystem.
vex::motor primaryLiftAMotor(vex::PORT9, false);
vex::motor primaryLiftBMotor(vex::PORT10, true);

// Single-motor mechanisms. Their native positive directions already match the
// module conventions: Toggle up, Secondary Lift raise, Claw close.
vex::motor toggleMechanismMotor(vex::PORT13, false);
vex::motor secondaryLiftMechanismMotor(vex::PORT5, false);
vex::motor clawMechanismMotor(vex::PORT6, false);

}  // namespace

namespace RobotHardware {

MotorPair driveFrontLeft = {&driveFrontLeftAMotor, &driveFrontLeftBMotor};
MotorPair driveFrontRight = {&driveFrontRightAMotor, &driveFrontRightBMotor};
MotorPair driveRearLeft = {&driveRearLeftAMotor, &driveRearLeftBMotor};
MotorPair driveRearRight = {&driveRearRightAMotor, &driveRearRightBMotor};
MotorPair primaryLift = {&primaryLiftAMotor, &primaryLiftBMotor};

vex::motor* toggleMotor = &toggleMechanismMotor;
vex::motor* secondaryLiftMotor = &secondaryLiftMechanismMotor;
vex::motor* clawMotor = &clawMechanismMotor;

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
