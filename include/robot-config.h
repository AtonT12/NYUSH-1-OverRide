#pragma once

#include "constants.h"
#include "vex.h"

// Confirmed V5 devices that do not require a Smart Port assignment.
extern vex::brain Brain;
extern vex::controller Controller;

// Central hardware bindings.
//
// Robot convention: the Claw side is the front; the Toggle side is the rear.
// Smart Ports, internal cartridge settings and current per-motor reverse flags
// are declared in robot-config.cpp. RPM commands target motor-shaft velocity;
// external drivetrain ratios and physical-distance conversions are not used.
namespace RobotHardware {

// Runtime operating mode. Defined in robot-config.cpp and deliberately not
// constexpr: a compile-time constant would let the optimiser delete every
// actuator call in the safe modes, which is exactly what happened when this was
// a `constexpr bool`.
extern RobotConstants::RunMode gRunMode;

// Brake modes and per-motor torque limits. Call once at startup, before any
// command can be issued.
void applyMotorConfiguration();

struct MotorPair {
  vex::motor* motorA;
  vex::motor* motorB;
};

// Four Omni X-drive wheel groups. Each wheel is driven by two motors through a
// gear transmission. Software direction is normalized per motor in
// robot-config.cpp; physical wheel response still needs validation.
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
