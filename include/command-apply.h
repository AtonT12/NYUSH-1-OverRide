#pragma once

#include "control/command.h"

// Single dispatcher from a normalized command to every actuator. Teleop and
// autonomous both call this, so a recorded run and a driven run cannot diverge
// because one of them forgot to update a mechanism.
namespace CommandApply {

void apply(const control::UserCommand& command);

}  // namespace CommandApply
