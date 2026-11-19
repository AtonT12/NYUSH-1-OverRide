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

// Control speeds in motor RPM, taken verbatim from the reference's execute()
// (NYU-Vex-2026 src/main.rs:257-262). Axis mapping there and here: "primary"
// is the two-motor L1/L2 lift on ports 9/10 (reference lift_fine), "secondary"
// is the single port-5 motor on Axis2 (reference lift_coarse).
constexpr double kDriveTranslationRpm = 600.0;  // MOVEMENT_SPEED
constexpr double kDriveTurnRpm = 400.0;         // ROTATE_SPEED
constexpr double kPrimaryLiftRpm = 100.0;       // LIFT_FINE_SPEED, ports 9,10
constexpr double kSecondaryLiftRpm = 50.0;      // LIFT_COARSE_SPEED, port 5
constexpr double kClawRpm = 150.0;              // FIXTURE_SPEED, port 6
constexpr double kToggleRpm = 100.0;            // TOGGLE_SPEED, port 13

// Output ceilings in RPM. Both are the free speed of the installed cartridge,
// so they only bite when a mixed command would ask for more than the hardware
// can deliver: 600 for the 6:1 drive, 200 for the 18:1 claw and toggle. The
// 36:1 lifts never reach the mechanism ceiling (100 RPM free speed).
//
// The reference clips each wheel independently instead; clampWheels scales all
// four proportionally so a saturated command keeps its heading.
constexpr double kDriveMaxCommandRpm = 600.0;
constexpr double kMechanismMaxCommandRpm = 200.0;

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

}  // namespace RobotConstants
