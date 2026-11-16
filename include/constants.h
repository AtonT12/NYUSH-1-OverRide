#pragma once

// Project-wide control parameters.
// Values here are software limits, not robot geometry or motor configuration.
namespace RobotConstants {

// Teleop loop period. Long enough to avoid wasting CPU, short enough for
// predictable operator response.
constexpr int kControlLoopPeriodMs = 20;

// Controller values within this threshold are zero; the rest is rescaled.
constexpr int kJoystickDeadband = 5;

// Control speeds adapted from the Python reference in motor RPM.
constexpr double kDriveTranslationRpm = 500.0;
constexpr double kDriveTurnRpm = 200.0;
constexpr double kPrimaryLiftRpm = 20.0;
constexpr double kClawRpm = 50.0;
constexpr double kToggleRpm = 25.0;

// Retain the project's conservative output caps in RPM: 80% of 600 RPM for
// the 6:1 drive cartridge and 60% of 200 RPM for the 18:1 mechanisms.
constexpr double kDriveMaxCommandRpm = 480.0;
constexpr double kMechanismMaxCommandRpm = 120.0;

// Keep actuator commands locked until motor directions have been bench-tested.
constexpr bool kDriverActuationEnabled = false;

}  // namespace RobotConstants
