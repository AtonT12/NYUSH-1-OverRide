#pragma once

#include "vex.h"

// Small, shared helpers for safe open-loop motor control.
// This is intentionally not a PID or motion-profile implementation.
namespace MotorControl {

// Returns zero for inputs inside the deadband, otherwise preserves the
// controller value (which is already in the -100..100 range).
int applyDeadband(int value, int deadband);

// Symmetric magnitude limit. The sign is preserved.
double limitMagnitude(double value, double maxMagnitude);

// Null-safe percent command. A null pointer is a valid "hardware not bound"
// state. Near-zero commands stop the motor instead of issuing a zero spin.
void spinPercent(vex::motor* motor, double percent);

// Null-safe stop with an explicit brake mode.
void stopMotor(vex::motor* motor, vex::brakeType mode);

}  // namespace MotorControl
