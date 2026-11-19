# Autonomous: record, encode, replay

The autonomous period does not read the controller and does not read the SD card.
A 30 s routine lives in flash as a frame table, so a cold robot on the field runs
exactly the same motion as the one on the bench.

## 1. Record a run

1. Set `kBootRunMode` to `RobotConstants::RunMode::Record` in
   `include/constants.h` and rebuild.
2. Drive the path you want to become the routine. Recording starts when driver
   control starts and stops when it ends.
3. After the driver period the brain writes `auton.bin` to the SD card and shows
   `REC saved N frames`.
4. Set `kBootRunMode` back to `RunMode::Field`.

Recording is capped at 30 000 ms and 1500 frames, matching the VEX U autonomous
period: at one frame per 20 ms control tick, 1500 frames is exactly 30 s. Every
tick is stored, including ticks where nothing changed.

An earlier version merged identical consecutive commands by extending the
previous frame's timestamp instead. That moved an input change to the *end* of
its hold, so replay ramped into a continuous command over the whole hold and
turned a one-second button press into a single 20 ms pulse. The frame writer now
appends unconditionally; `tests/test_recorder.cpp` pins both cases.

## 2. Encode it

```bash
python tools/ovr_encode.py auton.bin
```

This rewrites `src/auton/programs.cpp` with a `constexpr` table. Rebuild and
download. `python tools/ovr_encode.py --selftest` checks the codec against the
same CRC vector and layout the firmware uses.

The generated file looks like:

```cpp
const control::AutonFrame kFrames[] = {
    {0u, 0, 0, 0, 0, 0u, 0u},
    {400u, 600, 0, 0, 0, 0u, 0u},
};
const control::AutonProgram kSelected(kFrames, 2u, 20u);
```

## 3. Test it on the bench

Hold **Y + Down** while powering the brain on. The routine runs once, then the
robot returns to normal boot. At a competition the same keypress is harmless:
`pre_auton` runs while the robot is field-disabled, so the motors are locked.

The screen shows `AUTO <t> / <duration> ms` while it runs and `NO AUTON` when the
table is empty.

## 4. File format

16-byte header, then 12 bytes per frame, little-endian.

| Offset | Size | Content |
| --- | --- | --- |
| 0 | 4 | `OVRR` |
| 4 | 1 | version (1) |
| 5 | 1 | reserved |
| 6 | 2 | control period, ms |
| 8 | 2 | frame count |
| 10 | 2 | reserved |
| 12 | 4 | CRC-32 of the frame bytes |

| Frame offset | Size | Content |
| --- | --- | --- |
| 0 | 2 | timestamp, ms |
| 2 | 2 | forward, permille |
| 4 | 2 | strafe, permille |
| 6 | 2 | turn, permille |
| 8 | 2 | coarse lift, permille |
| 10 | 1 | flags |
| 11 | 1 | reserved |

Flag bits: 0 lift-fine up, 1 lift-fine down, 2 fixture up, 3 fixture down,
4 toggle, 5 end of program.

## 5. Why the replay does not drift

The runner samples on an absolute grid: tick `i` is always played at
`start + i * period`, regardless of how long the previous tick took. `period`
comes from the tick grid's own fallback value, not from the raw header field, so
a table with `period = 0` plays at 20 ms instead of freezing at `t = 0` forever.
A late loop skips frames instead of replaying them back-to-back, so a 30 s
routine ends within one period of 30 s instead of accumulating the loop's own
work over 1500 iterations. `tests/test_timebase.cpp` pins this with an irregular
7 ms poll.

## 6. What is deliberately not here

- **No SD read at runtime.** If the card is missing or corrupt, the routine still
  runs; only re-encoding needs the card.
- **No controller read.** Competition control blocks controller access during
  autonomous, so a routine that depended on it would do nothing on the field.
- **No position feedback.** The table is open loop; a routine that pushes into a
  wall will not notice. Closing that loop needs the mechanism axes in `C`.
