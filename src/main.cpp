/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Aton                                                      */
/*    Created:      2026/11/13 19:57:15                                       */
/*    Description:  VEX U Override competition program                        */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

#include "auton-programs.h"
#include "auton-runner.h"
#include "claw.h"
#include "command-apply.h"
#include "constants.h"
#include "control/shaping.h"
#include "control/timebase.h"
#include "diagnostics.h"
#include "drivetrain.h"
#include "recorder.h"
#include "robot-config.h"
#include "toggle.h"
#include "vertical-lift.h"

using namespace vex;

// A global instance of competition.
competition Competition;

namespace {

void stopAllActuators() {
  RobotHardware::stopAllMotors(vex::brakeType::brake);
}

void showRobotStatus() {
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("NYUSH-1 | VEX U Override");
  Brain.Screen.setCursor(2, 1);
  Brain.Screen.print("Small Robot | 8M Omni X-Drive");

  Brain.Screen.setCursor(3, 1);
  if (RobotHardware::allReady()) {
    Brain.Screen.print("HW bindings: ready");
  } else {
    Brain.Screen.print("HW bindings: not ready");
  }

  Brain.Screen.setCursor(4, 1);
  Brain.Screen.print("X-drive mixer: RPM");
  Brain.Screen.setCursor(5, 1);
  if (RobotHardware::gRunMode != RobotConstants::RunMode::Locked) {
    Brain.Screen.print("Driver output: enabled");
  } else {
    Brain.Screen.print("Driver output: locked");
  }
  Brain.Screen.setCursor(6, 1);
  Brain.Screen.print("Auton: %d frames, %d ms",
                     static_cast<int>(AutonPrograms::kSelected.count),
                     static_cast<int>(
                         control::autonDurationMs(AutonPrograms::kSelected)));
  Brain.Screen.setCursor(7, 1);
  Brain.Screen.print("Boot: X+B diag | Y+Down auto");
}

}  // namespace

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  stopAllActuators();

  // Encoder zeroing is safe only when the drivetrain is fully bound and at
  // rest. The module checks readiness before touching the motors.
  Drivetrain::resetEncoders();

  showRobotStatus();

  // Bench escape hatch: X + B held at power-on. Two buttons on purpose, so a
  // stray press at a competition cannot strand the robot in diagnostics.
  if (Controller.ButtonX.pressing() && Controller.ButtonB.pressing()) {
    Diagnostics::run();
    // Jogging moved the wheels, so the zero taken above is stale.
    Drivetrain::resetEncoders();
    showRobotStatus();
  }

  // Bench trigger for the autonomous table: Y + Down at power-on. At a
  // competition this runs while the robot is field-disabled, so nothing moves;
  // on the bench it is how a routine gets timed before it matters.
  if (Controller.ButtonY.pressing() && Controller.ButtonDown.pressing()) {
    AutonRunner::run(AutonPrograms::kSelected);
    stopAllActuators();
    showRobotStatus();
  }

  // No IMU or other sensor calibration is performed because no such sensor is
  // confirmed in the current hardware specification.
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  stopAllActuators();
  AutonRunner::run(AutonPrograms::kSelected);
  stopAllActuators();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  stopAllActuators();

  control::ShapingCal shaping;
  shaping.translationDeadband = 0.0;  // team decision: full stick resolution
  shaping.turnDeadband = RobotConstants::kTurnDeadband;
  shaping.liftCoarseDeadband = RobotConstants::kLiftCoarseDeadband;
  shaping.translationSlewPerSec = RobotConstants::kTranslationSlewPerSec;
  shaping.turnSlewPerSec = RobotConstants::kTurnSlewPerSec;
  shaping.liftCoarseSlewPerSec = RobotConstants::kLiftCoarseSlewPerSec;

  control::SlewState slew;
  control::TickGrid grid;
  control::tickReset(&grid, vex::timer::system(),
                     static_cast<uint16_t>(RobotConstants::kControlLoopPeriodMs));

  const bool recording = RobotHardware::gRunMode == RobotConstants::RunMode::Record;
  if (recording) {
    Recorder::reset();
  }

  int displayCycle = 0;

  while (Competition.isDriverControl()) {
    // Absolute schedule: sleep only until the next grid tick. A relative wait
    // at the end of the loop folds the loop's own work into every period and
    // drifts across the 90 s of a match.
    const uint32_t nowMs = vex::timer::system();
    if (!control::tickDue(grid, nowMs)) {
      const int32_t remaining =
          static_cast<int32_t>(control::tickTime(grid) - nowMs);
      wait(remaining > 0 ? remaining : 1, vex::msec);
      continue;
    }
    const uint32_t consumed = control::tickAdvance(&grid, nowMs);
    // dt follows the schedule, not the wall clock: a skipped frame produces one
    // larger slew step instead of a stall followed by a jump.
    const double dtSec = static_cast<double>(consumed) *
                         static_cast<double>(RobotConstants::kControlLoopPeriodMs) /
                         1000.0;

    control::UserCommand raw;
    raw.forward = Controller.Axis3.position() / 100.0;
    raw.strafe = Controller.Axis4.position() / 100.0;
    raw.turn = Controller.Axis1.position() / 100.0;
    // Axis2 sign is unverified; bench test decides whether this needs a minus.
    raw.liftCoarse = Controller.Axis2.position() / 100.0;
    raw.liftFine =
        Controller.ButtonL1.pressing() ? 1 : (Controller.ButtonL2.pressing() ? -1 : 0);
    raw.fixture =
        Controller.ButtonR1.pressing() ? 1 : (Controller.ButtonR2.pressing() ? -1 : 0);
    raw.toggle = Controller.ButtonA.pressing() || Controller.ButtonX.pressing();

    const control::UserCommand command = control::shape(raw, &slew, shaping, dtSec);

    if (RobotHardware::gRunMode != RobotConstants::RunMode::Locked) {
      CommandApply::apply(command);
      if (recording) {
        Recorder::push(command, control::tickElapsed(grid, nowMs));
      }
    }

    if (++displayCycle >= 10) {
      displayCycle = 0;
      Brain.Screen.printAt(
          10, 210, "F:%4d S:%4d T:%4d %s",
          static_cast<int>(command.forward * 100.0),
          static_cast<int>(command.strafe * 100.0),
          static_cast<int>(command.turn * 100.0),
          RobotHardware::gRunMode != RobotConstants::RunMode::Locked ? "ON"
                                                                    : "LOCKED");
    }
  }

  // Cleanup path for leaving driver control: stop every bound motor.
  stopAllActuators();

  if (recording) {
    // Saved after the motors are stopped: an 18 KB SD write has no business
    // running between two control ticks.
    const bool saved = Recorder::save("auton.bin");
    Brain.Screen.printAt(10, 190,
                         saved ? "REC saved %d frames" : "REC FAILED %d frames",
                         static_cast<int>(Recorder::frameCount()));
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Brake modes and torque limits must be in place before any callback can
  // issue a command.
  RobotHardware::applyMotorConfiguration();

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
