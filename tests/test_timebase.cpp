#include "control/timebase.h"
#include "harness.h"

TEST_CASE(tick_time_starts_at_the_reset_instant) {
  control::TickGrid grid;
  control::tickReset(&grid, 1000u, 20u);
  CHECK_INT(control::tickTime(grid), 1000u);
}

TEST_CASE(tick_is_not_due_before_the_deadline) {
  control::TickGrid grid;
  control::tickReset(&grid, 1000u, 20u);
  CHECK_FALSE(control::tickDue(grid, 999u));
  CHECK_TRUE(control::tickDue(grid, 1000u));
}

TEST_CASE(tick_advance_consumes_one_at_the_deadline) {
  control::TickGrid grid;
  control::tickReset(&grid, 1000u, 20u);
  CHECK_INT(control::tickAdvance(&grid, 1000u), 1u);
  CHECK_INT(grid.index, 1u);
  CHECK_INT(control::tickTime(grid), 1020u);
}

TEST_CASE(tick_advance_returns_zero_when_early) {
  control::TickGrid grid;
  control::tickReset(&grid, 1000u, 20u);
  // The tick at t=1000 is pending by definition, so consume it first; the next
  // deadline is 1020 and anything before that must not advance the grid.
  CHECK_INT(control::tickAdvance(&grid, 1000u), 1u);
  CHECK_INT(control::tickAdvance(&grid, 1019u), 0u);
  CHECK_INT(grid.index, 1u);
}

TEST_CASE(tick_advance_skips_instead_of_catching_up) {
  control::TickGrid grid;
  control::tickReset(&grid, 1000u, 20u);
  // 85 ms late on a 20 ms grid: ticks at 1000/1020/1040/1060/1080 are all
  // missed, so five ticks are consumed and the next deadline is 1100.
  CHECK_INT(control::tickAdvance(&grid, 1085u), 5u);
  CHECK_INT(grid.index, 5u);
  CHECK_INT(control::tickTime(grid), 1100u);
}

TEST_CASE(tick_grid_survives_millisecond_counter_wrap) {
  control::TickGrid grid;
  control::tickReset(&grid, 0xFFFFFF00u, 20u);
  CHECK_TRUE(control::tickDue(grid, 0x00000010u));
  CHECK_INT(control::tickAdvance(&grid, 0x00000010u), 14u);
  CHECK_INT(control::tickTime(grid), 0x00000018u);
  CHECK_FALSE(control::tickDue(grid, 0x00000010u));
}

TEST_CASE(tick_elapsed_measures_from_the_reset_instant) {
  control::TickGrid grid;
  control::tickReset(&grid, 500u, 20u);
  CHECK_INT(control::tickElapsed(grid, 780u), 280u);
}

TEST_CASE(tick_reset_rejects_a_zero_period) {
  control::TickGrid grid;
  control::tickReset(&grid, 0u, 0u);
  CHECK_INT(grid.periodMs, 20u);
}

TEST_CASE(playback_stays_on_the_absolute_schedule_under_jitter) {
  // Mimics the autonomous runner: an irregular poll interval, one command per
  // consumed tick, sampling at (index - 1) * period. After 5000 ms of wall
  // clock the newest command must still belong to the 5000 ms tick; a loop
  // built on relative sleeps would have drifted by dozens of milliseconds.
  control::TickGrid grid;
  control::tickReset(&grid, 1000u, 20u);

  uint32_t lastNowMs = 1000u;
  uint32_t lastTickMs = 0;
  for (uint32_t now = 1000u; now <= 6000u; now += 7u) {
    if (!control::tickDue(grid, now)) {
      continue;
    }
    control::tickAdvance(&grid, now);
    lastTickMs = (grid.index - 1u) * 20u;
    lastNowMs = now;
  }

  // The newest executed tick may lag the wall clock by at most one period, and
  // it may never run ahead of it.
  const uint32_t elapsedMs = lastNowMs - 1000u;
  CHECK_TRUE(lastTickMs <= elapsedMs);
  CHECK_TRUE(lastTickMs + 20u >= elapsedMs);
}
