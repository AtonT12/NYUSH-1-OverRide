#pragma once

#include <stdint.h>

// Project-wide control parameters.
// Values here are software limits, not robot geometry or motor configuration.
namespace RobotConstants {

// Teleop loop period. Long enough to avoid wasting CPU, short enough for
// predictable operator response.
constexpr int kControlLoopPeriodMs = 20;

// Input shaping. Translation deliberately has no deadband: the team asked for
// full stick resolution on the drive axes and accepts that a worn stick may
// creep. Turn and the coarse lift keep one because neither has a visual
// reference to trim against.
constexpr double kTurnDeadband = 0.20;
constexpr double kLiftCoarseDeadband = 0.20;

// Slew rates in normalized units per second, not per frame: changing the loop
// period must not change the feel. The reference steps 0.1 and 0.05 per frame at
// Controller::UPDATE_INTERVAL = 25 ms, so the equivalent rates are 4/s and 2/s.
// The reference does not shape the coarse lift, so it is left at 0 here.
constexpr double kTranslationSlewPerSec = 4.0;
constexpr double kTurnSlewPerSec = 2.0;
constexpr double kLiftCoarseSlewPerSec = 0.0;

// Control speeds adapted from the Python reference in motor RPM.
constexpr double kDriveTranslationRpm = 500.0;
constexpr double kDriveTurnRpm = 200.0;
constexpr double kPrimaryLiftRpm = 20.0;
constexpr double kSecondaryLiftRpm = 50.0;
constexpr double kClawRpm = 50.0;
constexpr double kToggleRpm = 25.0;

// Retain the project's conservative output caps in RPM: 80% of 600 RPM for
// the 6:1 drive cartridge and 60% of 200 RPM for the 18:1 mechanisms.
constexpr double kDriveMaxCommandRpm = 480.0;
constexpr double kMechanismMaxCommandRpm = 120.0;

// Operating mode. This used to be a single `constexpr bool`, which the compiler
// folded to `false` and then used to delete the entire teleop path: the built
// binary contained no drivetrain code at all. The mode is now a runtime value
// held in robot-config.cpp, so the drive code is always compiled and linked.
enum class RunMode : uint8_t {
  Locked,  // motors stopped, sticks only displayed
  Bench,   // drivetrain capped to kBenchWheelCapRpm for direction checks
  Field,   // full competition output
  Record   // Field, plus writing a programmed run to the SD card
};

constexpr RunMode kBootRunMode = RunMode::Field;

// Bench cap: slow enough to read a wheel by eye, fast enough to prove direction.
constexpr double kBenchWheelCapRpm = 60.0;

// Current limits. Eight drive motors on one battery will brown out the brain
// long before they reach the motor's own limit; these keep headroom for the
// mechanisms.
constexpr double kDriveTorquePercent = 60.0;
constexpr double kMechanismTorquePercent = 40.0;

}  // namespace RobotConstants
