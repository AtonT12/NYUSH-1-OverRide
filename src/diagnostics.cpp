#include "diagnostics.h"

#include "constants.h"
#include "robot-config.h"

namespace {

struct Entry {
  const char* label;
  int port;
  vex::motor* motor;
};

// One row per motor, not per wheel group: the sign-off table in
// docs/hardware.md is per motor, and a pair can hide a single reversed member.
Entry gEntries[] = {
    {"FL-A", 1, RobotHardware::driveFrontLeft.motorA},
    {"FL-B", 2, RobotHardware::driveFrontLeft.motorB},
    {"FR-A", 4, RobotHardware::driveFrontRight.motorA},
    {"FR-B", 14, RobotHardware::driveFrontRight.motorB},
    {"RL-A", 7, RobotHardware::driveRearLeft.motorA},
    {"RL-B", 8, RobotHardware::driveRearLeft.motorB},
    {"RR-A", 3, RobotHardware::driveRearRight.motorA},
    {"RR-B", 19, RobotHardware::driveRearRight.motorB},
    {"LIFT-A", 9, RobotHardware::primaryLift.motorA},
    {"LIFT-B", 10, RobotHardware::primaryLift.motorB},
    {"TOGGLE", 13, RobotHardware::toggleMotor},
    {"LIFT2", 5, RobotHardware::secondaryLiftMotor},
    {"CLAW", 6, RobotHardware::clawMotor},
};

const int kCount = static_cast<int>(sizeof(gEntries) / sizeof(gEntries[0]));

// Low enough to read a wheel by eye, high enough that a dead motor is obvious.
const int kJogRpm = 60;

void stopSelection(const Entry& entry) {
  if (entry.motor != 0) {
    entry.motor->stop(vex::brakeType::brake);
  }
}

void draw(int index, int command) {
  const Entry& entry = gEntries[index];
  const bool installed = entry.motor != 0 && entry.motor->installed();
  const int positionDeg =
      entry.motor != 0 ? static_cast<int>(entry.motor->position(vex::degrees)) : 0;

  Brain.Screen.printAt(10, 20, "DIAG  %d / %d    port %d", index + 1, kCount,
                       entry.port);
  Brain.Screen.printAt(10, 52, false, "%s", entry.label);
  Brain.Screen.printAt(10, 82, false, "state   %s", installed ? "ready" : "NOT FOUND");
  Brain.Screen.printAt(10, 112, "cmd     %d rpm", command);
  Brain.Screen.printAt(10, 142, "pos     %d deg", positionDeg);
  Brain.Screen.printAt(10, 182, "A +   B -   X stop");
  Brain.Screen.printAt(10, 208, "Y exit");
}

}  // namespace

namespace Diagnostics {

void run() {
  // Nothing should be spinning when the tool opens, and the previous mode's
  // brake configuration may differ from the one this loop needs.
  RobotHardware::stopAllMotors(vex::brakeType::brake);

  int index = 0;
  int command = 0;
  // Seed from the live state so a button already held at entry is not treated
  // as a fresh press.
  bool previousUp = Controller.ButtonUp.pressing();
  bool previousDown = Controller.ButtonDown.pressing();
  bool previousExit = Controller.ButtonY.pressing();
  int redrawCounter = 0;

  Brain.Screen.clearScreen();
  draw(index, command);

  while (true) {
    const bool up = Controller.ButtonUp.pressing();
    const bool down = Controller.ButtonDown.pressing();
    const bool exitRequested = Controller.ButtonY.pressing();

    if (up && !previousUp) {
      stopSelection(gEntries[index]);
      index = (index + kCount - 1) % kCount;
      command = 0;
      redrawCounter = 100;
    }
    if (down && !previousDown) {
      stopSelection(gEntries[index]);
      index = (index + 1) % kCount;
      command = 0;
      redrawCounter = 100;
    }

    const bool forward = Controller.ButtonA.pressing();
    const bool reverse = Controller.ButtonB.pressing();
    int nextCommand = 0;
    if (forward && !reverse) {
      nextCommand = kJogRpm;
    } else if (reverse && !forward) {
      nextCommand = -kJogRpm;
    }

    Entry& entry = gEntries[index];
    const bool installed = entry.motor != 0 && entry.motor->installed();
    if (!installed) {
      nextCommand = 0;
    }

    if (nextCommand != command) {
      command = nextCommand;
      if (command == 0) {
        stopSelection(entry);
      } else if (command > 0) {
        entry.motor->spin(vex::directionType::fwd, command, vex::velocityUnits::rpm);
      } else {
        entry.motor->spin(vex::directionType::rev, -command, vex::velocityUnits::rpm);
      }
      redrawCounter = 100;
    }

    // 100 ms redraw: fast enough to watch the position count, slow enough not
    // to spend the loop inside the screen driver.
    if (redrawCounter >= 5) {
      redrawCounter = 0;
      draw(index, command);
    }
    ++redrawCounter;

    previousUp = up;
    previousDown = down;

    if (exitRequested && !previousExit) {
      break;
    }
    previousExit = exitRequested;

    wait(RobotConstants::kControlLoopPeriodMs, vex::msec);
  }

  RobotHardware::stopAllMotors(vex::brakeType::brake);
  Brain.Screen.clearScreen();
}

}  // namespace Diagnostics
