#include "robot-config.h"

#include "motor-control.h"

// Brain and Controller are confirmed hardware.
vex::brain Brain;
vex::controller Controller;

namespace {

// Port map and current motor-direction assumptions.
//
// Robot convention: the Claw is the front, the Toggle is the rear. Each drive
// group is normalized so a positive module command drives that wheel forward.
// Native-positive directions were supplied in the original hardware notes.
// The Python reference has opposite flags for all drive motors, so those
// directions still require a supervised physical check.
//
// Internal V5 gear cartridges follow the hardware configuration in the Python
// reference: 6:1 for drive motors and 18:1 for mechanisms.

// FL native positive = backward -> invert to make positive = forward.
vex::motor driveFrontLeftAMotor(vex::PORT1, vex::ratio6_1, true);
vex::motor driveFrontLeftBMotor(vex::PORT2, vex::ratio6_1, true);

// FR native positive = forward.
vex::motor driveFrontRightAMotor(vex::PORT4, vex::ratio6_1, false);
vex::motor driveFrontRightBMotor(vex::PORT14, vex::ratio6_1, false);

// RL native positive = backward -> invert to make positive = forward.
vex::motor driveRearLeftAMotor(vex::PORT7, vex::ratio6_1, true);
vex::motor driveRearLeftBMotor(vex::PORT8, vex::ratio6_1, true);

// RR native positive = forward.
vex::motor driveRearRightAMotor(vex::PORT3, vex::ratio6_1, false);
vex::motor driveRearRightBMotor(vex::PORT19, vex::ratio6_1, false);

// Lift A native positive = raise. Lift B native positive = lower, so invert B
// to make positive command = raise for the complete subsystem.
vex::motor primaryLiftAMotor(vex::PORT9, vex::ratio18_1, false);
vex::motor primaryLiftBMotor(vex::PORT10, vex::ratio18_1, true);

// Single-motor mechanisms. Their native positive directions already match the
// module conventions: Toggle up, Secondary Lift raise, Claw close.
vex::motor toggleMechanismMotor(vex::PORT13, vex::ratio18_1, false);
vex::motor secondaryLiftMechanismMotor(vex::PORT5, vex::ratio18_1, false);
vex::motor clawMechanismMotor(vex::PORT6, vex::ratio18_1, false);

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

// A motor counts as ready only when the pointer is bound (guards code edits)
// and the device actually answers on its Smart Port. Without installed() this
// check only proves that the source code declared a motor object, which is
// always true and would report "ready" with the whole robot unplugged.
bool motorReady(vex::motor* motor) {
  return motor != nullptr && motor->installed();
}

bool pairReady(const MotorPair& pair) {
  return motorReady(pair.motorA) && motorReady(pair.motorB);
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
  return motorReady(toggleMotor);
}

bool primaryLiftReady() {
  return pairReady(primaryLift);
}

bool secondaryLiftReady() {
  return motorReady(secondaryLiftMotor);
}

bool clawReady() {
  return motorReady(clawMotor);
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
