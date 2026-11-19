#include "auton-runner.h"

#include "command-apply.h"
#include "constants.h"
#include "control/timebase.h"
#include "robot-config.h"
#include "vex.h"

namespace {

// VEX U plays a 30 s autonomous period (<VUT4>). The budget is enforced in
// software so a malformed table cannot run into driver control.
const uint32_t kMatchBudgetMs = 30000u;

}  // namespace

namespace AutonRunner {

void run(const control::AutonProgram& program) {
  Brain.Screen.clearScreen();

  if (program.frames == 0 || program.count == 0) {
    Brain.Screen.printAt(10, 40, "NO AUTON");
    return;
  }
  if (RobotHardware::gRunMode == RobotConstants::RunMode::Locked) {
    Brain.Screen.printAt(10, 40, "AUTON BLOCKED: LOCKED");
    return;
  }

  const uint32_t durationMs = control::autonDurationMs(program);
  const uint32_t limitMs = durationMs < kMatchBudgetMs ? durationMs : kMatchBudgetMs;

  control::TickGrid grid;
  control::tickReset(&grid, vex::timer::system(), program.periodMs);

  int displayCounter = 0;

  while (true) {
    const uint32_t nowMs = vex::timer::system();
    if (!control::tickDue(grid, nowMs)) {
      const int32_t remaining =
          static_cast<int32_t>(control::tickTime(grid) - nowMs);
      wait(remaining > 0 ? remaining : 1, vex::msec);
      continue;
    }

    // Sample on the ideal grid rather than on the wall clock. A late loop must
    // not stretch the routine, and frames missed while the CPU was busy are
    // skipped rather than replayed back-to-back.
    control::tickAdvance(&grid, nowMs);
    const uint32_t tMs = (grid.index - 1u) * static_cast<uint32_t>(program.periodMs);
    if (tMs > limitMs) {
      break;
    }

    CommandApply::apply(control::sampleAuton(program, tMs));

    if (++displayCounter >= 10) {
      displayCounter = 0;
      Brain.Screen.printAt(10, 220, "AUTO %d / %d ms", static_cast<int>(tMs),
                           static_cast<int>(limitMs));
    }
  }

  Brain.Screen.printAt(10, 220, "AUTO done %d ms", static_cast<int>(limitMs));
}

}  // namespace AutonRunner
