#pragma once

#include "control/auton.h"

// The autonomous table the robot runs. Regenerate with:
//   python tools/ovr_encode.py <recording>.bin
// The checked-in version is empty, so a fresh clone reports NO AUTON instead of
// driving off with someone else's routine.
namespace AutonPrograms {

extern const control::AutonProgram kSelected;

}  // namespace AutonPrograms
