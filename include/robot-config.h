#pragma once

#include "vex.h"

// Confirmed V5 devices that do not require a Smart Port assignment.
extern vex::brain Brain;
extern vex::controller Controller;

// Central hardware bindings.
//
// Robot convention: the Claw side is the front; the Toggle side is the rear.
// The confirmed Smart Ports and per-motor reverse flags are declared in
// robot-config.cpp. Cartridges remain unconfirmed, so the modules use percent
// power only and do not perform RPM or physical-distance conversions.
namespace RobotHardware {

struct MotorPair {
  vex::motor* motorA;
  vex::motor* motorB;
};

// Four Omni X-drive wheel groups. Each wheel is driven by two motors through a
// gear transmission. Direction is normalized per motor in robot-config.cpp so a
// positive Drivetrain command drives its wheel group forward.
extern MotorPair driveFrontLeft;
extern MotorPair driveFrontRight;
extern MotorPair driveRearLeft;
extern MotorPair driveRearRight;

// Primary vertical lift: two motors treated as one logical subsystem. Direction
// is normalized so positive VerticalLift power raises the assembly.
extern MotorPair primaryLift;

// Single-motor mechanisms.
extern vex::motor* toggleMotor;
extern vex::motor* secondaryLiftMotor;
extern vex::motor* clawMotor;

// Readiness is true only when every motor needed by that subsystem is bound.
// A partially bound pair is intentionally treated as not ready.
bool drivetrainReady();
bool toggleReady();
bool primaryLiftReady();
bool secondaryLiftReady();
bool clawReady();
bool allReady();

// Stops every bound motor, including any partially configured pair. Safe to
// call when no motor is bound.
void stopAllMotors(vex::brakeType mode);

}  // namespace RobotHardware
