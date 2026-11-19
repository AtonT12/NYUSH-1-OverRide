#pragma once

#include <stdint.h>

namespace control {

// The single command format shared by teleop, recording and playback.
// Continuous axes are normalized to [-1, 1]; discrete channels are -1/0/+1.
//
// Having exactly one command type is what makes "record a driver run and play
// it back" trustworthy: the playback path cannot drift from the teleop path as
// long as both produce a UserCommand and both feed the same kinematics.
struct UserCommand {
  double forward;     // + = towards the claw side
  double strafe;      // + = to the robot's right
  double turn;        // + = clockwise seen from above
  double liftCoarse;  // + = raise
  int8_t liftFine;    // -1 / 0 / +1
  int8_t fixture;     // -1 / 0 / +1 (claw)
  bool toggle;

  constexpr UserCommand()
      : forward(0.0),
        strafe(0.0),
        turn(0.0),
        liftCoarse(0.0),
        liftFine(0),
        fixture(0),
        toggle(false) {}
};

}  // namespace control
