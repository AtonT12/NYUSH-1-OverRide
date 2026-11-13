#pragma once

#include "vex.h"

// Confirmed V5 devices that do not require a Smart Port assignment.
extern vex::brain Brain;
extern vex::controller Controller;

// Central hardware binding seam.
//
// The confirmed robot has 13 motors, but no Smart Port map, cartridge map or
// per-motor direction data has been supplied. Do not replace these null
// bindings with guessed values. Once the physical configuration is verified,
// declare the motor objects in robot-config.cpp (using the confirmed cartridge
// and an independent reverse flag per motor) and point these bindings at them.
namespace RobotHardware {

struct MotorPair {
  vex::motor* motorA;
  vex::motor* motorB;
};

// Four Omni X-drive wheel groups. Each wheel is driven by two motors through a
// gear transmission; the two motors are configured independently so opposite
// mounting directions can be corrected without changing module code.
extern MotorPair driveFrontLeft;
extern MotorPair driveFrontRight;
extern MotorPair driveRearLeft;
extern MotorPair driveRearRight;

// Primary vertical lift: two motors treated as one logical subsystem.
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

// Stops every bound motor, including motors in partially configured pairs.
// Safe to call when nothing is bound.
void stopAllMotors(vex::brakeType mode);

}  // namespace RobotHardware
