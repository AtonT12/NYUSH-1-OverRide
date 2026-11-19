#pragma once

#include "robot-config.h"

// One logical mechanism axis: one or two motors that always receive the same
// command. Replaces four near-identical modules that each repeated
// readiness -> clamp -> spin -> stop.
namespace Mechanism {

bool isReady(const RobotHardware::MotorPair& axis);

// Symmetric command in RPM, clamped to maxRpm. A near-zero command stops the
// axis with its configured default mode (hold for every mechanism), which an
// explicit stop(brake) would override.
void setRpm(const RobotHardware::MotorPair& axis, double rpm, double maxRpm);

// Stops the axis with the configured default mode.
void stop(const RobotHardware::MotorPair& axis);

}  // namespace Mechanism
