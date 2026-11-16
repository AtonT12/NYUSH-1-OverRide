/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Aton                                                      */
/*    Created:      2026/11/13 19:57:15                                       */
/*    Description:  VEX U Override competition program                        */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

#include "autonomous.h"
#include "claw.h"
#include "constants.h"
#include "drivetrain.h"
#include "motor-control.h"
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
  if (RobotConstants::kDriverActuationEnabled) {
    Brain.Screen.print("Driver output: enabled");
  } else {
    Brain.Screen.print("Driver output: locked");
  }
  Brain.Screen.setCursor(6, 1);
  Brain.Screen.print("Auton: no routine");
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
  Autonomous::run();
  stopAllActuators();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  stopAllActuators();

  int displayCycle = 0;

  while (1) {
    // Once driver control ends, exit so the cleanup below can run instead of
    // leaving the last command state in place after a field disable.
    if (!Competition.isDriverControl()) {
      break;
    }

    const int forward = MotorControl::applyDeadband(
        Controller.Axis3.position(), RobotConstants::kJoystickDeadband);
    const int strafe = MotorControl::applyDeadband(
        Controller.Axis4.position(), RobotConstants::kJoystickDeadband);
    const int turn = MotorControl::applyDeadband(
        Controller.Axis1.position(), RobotConstants::kJoystickDeadband);

    if (RobotConstants::kDriverActuationEnabled) {
      Drivetrain::driveX(forward, strafe, turn);

      if (Controller.ButtonL1.pressing()) {
        VerticalLift::setRpm(RobotConstants::kPrimaryLiftRpm);
      } else if (Controller.ButtonL2.pressing()) {
        VerticalLift::setRpm(-RobotConstants::kPrimaryLiftRpm);
      } else {
        VerticalLift::stop();
      }

      if (Controller.ButtonR1.pressing()) {
        Claw::setRpm(RobotConstants::kClawRpm);
      } else if (Controller.ButtonR2.pressing()) {
        Claw::setRpm(-RobotConstants::kClawRpm);
      } else {
        Claw::stop();
      }

      if (Controller.ButtonA.pressing() || Controller.ButtonX.pressing()) {
        Toggle::setRpm(RobotConstants::kToggleRpm);
      } else {
        Toggle::stop();
      }
    }

    if (++displayCycle >= 10) {
      displayCycle = 0;
      Brain.Screen.printAt(10, 210, "F:%4d S:%4d T:%4d %s", forward, strafe,
                           turn, RobotConstants::kDriverActuationEnabled
                                     ? "ON"
                                     : "LOCKED");
    }

    wait(RobotConstants::kControlLoopPeriodMs, msec);
  }

  // Cleanup path for leaving driver control: stop every bound motor.
  stopAllActuators();
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
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
