#pragma once

#include "vex.h"

// Small, shared helpers for safe open-loop motor control.
// This is intentionally not a PID or motion-profile implementation.
namespace MotorControl {

// Symmetric magnitude limit in the caller's units. The sign is preserved.
double limitMagnitude(double value, double maxMagnitude);

// Null-safe RPM command. A null pointer is a valid "hardware not bound"
// state. Near-zero commands stop the motor instead of issuing a zero spin.
void spinRpm(vex::motor* motor, double rpm);

// Null-safe stop with an explicit brake mode.
void stopMotor(vex::motor* motor, vex::brakeType mode);

}  // namespace MotorControl
