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

// Mechanism axes. A single-motor axis sets motorB to nullptr; that is a
// complete axis, not a missing motor, which is why axisReady() accepts it.
// primaryLift is the L1/L2 pair on ports 9 and 10, secondaryLift is the port-5
// motor driven by Axis2.
extern MotorPair primaryLift;
extern MotorPair secondaryLift;
extern MotorPair claw;
extern MotorPair toggle;

// Readiness. A drive group needs both motors; a mechanism axis needs motorA and
// treats a null motorB as "this axis has one motor".
bool pairReady(const MotorPair& pair);
bool axisReady(const MotorPair& axis);

bool drivetrainReady();
bool toggleReady();
bool primaryLiftReady();
bool secondaryLiftReady();
bool clawReady();
bool allReady();

// Stops every bound motor. Drive groups brake; mechanism axes stop with their
// configured default mode (hold), so a lifted object does not drop. Safe to
// call when no motor is bound.
void stopAllMotors();

}  // namespace RobotHardware
