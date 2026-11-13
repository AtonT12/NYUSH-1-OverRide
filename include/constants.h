#pragma once

// Project-wide control parameters.
// Values here are software limits, not robot geometry or motor configuration.
namespace RobotConstants {

// Teleop loop period. Long enough to avoid wasting CPU, short enough for
// predictable operator response.
constexpr int kControlLoopPeriodMs = 20;

// Controller axis values within +/- this threshold are treated as zero.
constexpr int kJoystickDeadband = 5;

// Safety limits applied by the hardware modules. These can be tuned after the
// real mechanisms are characterized, but must remain conservative initially.
constexpr double kDriveMaxPercent = 80.0;
constexpr double kMechanismMaxPercent = 60.0;

}  // namespace RobotConstants
