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
// reference: 6:1 for drive motors, 36:1 for both lift axes, 18:1 for the claw
// and toggle. The gearset argument converts RPM commands into motor duty, so a
// wrong value silently halves or doubles the real output speed.

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
vex::motor primaryLiftAMotor(vex::PORT9, vex::ratio36_1, false);
vex::motor primaryLiftBMotor(vex::PORT10, vex::ratio36_1, true);

// Single-motor mechanisms. Their native positive directions already match the
// module conventions: Toggle up, Secondary Lift raise, Claw close.
vex::motor toggleMechanismMotor(vex::PORT13, vex::ratio18_1, false);
vex::motor secondaryLiftMechanismMotor(vex::PORT5, vex::ratio36_1, false);
vex::motor clawMechanismMotor(vex::PORT6, vex::ratio18_1, false);

}  // namespace

namespace RobotHardware {

RobotConstants::RunMode gRunMode = RobotConstants::kBootRunMode;

MotorPair driveFrontLeft = {&driveFrontLeftAMotor, &driveFrontLeftBMotor};
MotorPair driveFrontRight = {&driveFrontRightAMotor, &driveFrontRightBMotor};
MotorPair driveRearLeft = {&driveRearLeftAMotor, &driveRearLeftBMotor};
MotorPair driveRearRight = {&driveRearRightAMotor, &driveRearRightBMotor};
MotorPair primaryLift = {&primaryLiftAMotor, &primaryLiftBMotor};

MotorPair secondaryLift = {&secondaryLiftMechanismMotor, nullptr};
MotorPair claw = {&clawMechanismMotor, nullptr};
MotorPair toggle = {&toggleMechanismMotor, nullptr};

namespace {

// A motor counts as ready only when the pointer is bound (guards code edits)
// and the device actually answers on its Smart Port. Without installed() this
// check only proves that the source code declared a motor object, which is
// always true and would report "ready" with the whole robot unplugged.
bool motorReady(vex::motor* motor) {
  return motor != nullptr && motor->installed();
}

void stopPair(const MotorPair& pair, vex::brakeType mode) {
  MotorControl::stopMotor(pair.motorA, mode);
  MotorControl::stopMotor(pair.motorB, mode);
}

void stopAxis(const MotorPair& axis) {
  // No explicit mode: this is the default stopping mode, hold for mechanisms.
  if (axis.motorA != nullptr) {
    axis.motorA->stop();
  }
  if (axis.motorB != nullptr) {
    axis.motorB->stop();
  }
}

}  // namespace

bool pairReady(const MotorPair& pair) {
  return motorReady(pair.motorA) && motorReady(pair.motorB);
}

bool axisReady(const MotorPair& axis) {
  // A null motorB marks a single-motor axis, not a missing one.
  return motorReady(axis.motorA) &&
         (axis.motorB == nullptr || motorReady(axis.motorB));
}

bool drivetrainReady() {
  return pairReady(driveFrontLeft) && pairReady(driveFrontRight) &&
         pairReady(driveRearLeft) && pairReady(driveRearRight);
}

bool toggleReady() {
  return axisReady(toggle);
}

bool primaryLiftReady() {
  return axisReady(primaryLift);
}

bool secondaryLiftReady() {
  return axisReady(secondaryLift);
}

bool clawReady() {
  return axisReady(claw);
}

bool allReady() {
  return drivetrainReady() && toggleReady() && primaryLiftReady() &&
         secondaryLiftReady() && clawReady();
}

void stopAllMotors() {
  stopPair(driveFrontLeft, vex::brakeType::brake);
  stopPair(driveFrontRight, vex::brakeType::brake);
  stopPair(driveRearLeft, vex::brakeType::brake);
  stopPair(driveRearRight, vex::brakeType::brake);

  stopAxis(primaryLift);
  stopAxis(secondaryLift);
  stopAxis(claw);
  stopAxis(toggle);
}

void applyMotorConfiguration() {
  // Drive wheels brake on release; mechanisms hold, otherwise a lifted object
  // drops the moment the operator lets go of the button. There is deliberately
  // no setMaxTorque call: the reference drives every motor at its own current
  // limit, and a partial limit is felt as "the whole robot is weak".
  vex::motor* const driveMotors[] = {
      &driveFrontLeftAMotor,  &driveFrontLeftBMotor, &driveFrontRightAMotor,
      &driveFrontRightBMotor, &driveRearLeftAMotor,  &driveRearLeftBMotor,
      &driveRearRightAMotor,  &driveRearRightBMotor,
  };
  vex::motor* const mechanismMotors[] = {
      &primaryLiftAMotor, &primaryLiftBMotor, &toggleMechanismMotor,
      &secondaryLiftMechanismMotor, &clawMechanismMotor,
  };

  const unsigned driveCount = sizeof(driveMotors) / sizeof(driveMotors[0]);
  for (unsigned i = 0; i < driveCount; ++i) {
    driveMotors[i]->setStopping(vex::brakeType::brake);
  }

  const unsigned mechanismCount =
      sizeof(mechanismMotors) / sizeof(mechanismMotors[0]);
  for (unsigned i = 0; i < mechanismCount; ++i) {
    mechanismMotors[i]->setStopping(vex::brakeType::hold);
  }
}

}  // namespace RobotHardware
