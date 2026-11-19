#pragma once

#include "control/auton.h"

// Plays a programmed run on the absolute control grid. No controller reads, no
// SD reads, no reliance on anything recorded earlier in the same power cycle:
// the table lives in flash, so a cold robot on the field runs the same routine.
namespace AutonRunner {

void run(const control::AutonProgram& program);

}  // namespace AutonRunner
