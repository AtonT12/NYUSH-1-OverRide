# NYUSH-1 — VEX U Override Small Robot

VEX V5 competition program in C++11, built with the VEXcode makefile.
13 motors: 8-motor Omni X-drive, fine and coarse lift, claw, toggle.

## Build

VEX SDK `V5_20240802_15_00_00`, program slot 1.

```bash
VEX="$HOME/AppData/Roaming/Code/User/globalStorage/vexrobotics.vexcode"
TOOL="$VEX/tools/cpp/toolchain_win32"
PATH="$TOOL/gcc/bin:$TOOL/tools/bin:$TOOL/clang/bin:$PATH" \
  make T="$VEX/sdk/cpp/V5/V5_20240802_15_00_00" P=NYUSH-1
```

Or use **Build** in the VEX extension. Output is `build/NYUSH-1.bin`.

## Host tests

```bash
bash tests/run_tests.sh
```

Needs only `g++`; no VEX SDK, no robot. Covers the mixer, input shaping, the
replay timebase and the recorded-program encoding.

## Run modes

Set `RobotConstants::kBootRunMode` in `include/constants.h`.

| Mode | Behaviour |
| --- | --- |
| `Locked` | motors stopped, stick values on screen |
| `Bench` | drivetrain capped to 60 rpm |
| `Field` | full competition output (boot default) |
| `Record` | `Field`, plus writing `auton.bin` to the SD card |

## Driver controls

| Input | Action |
| --- | --- |
| Left stick Axis3 / Axis4 | forward / strafe |
| Right stick Axis1 | turn |
| Axis2 | coarse lift (port 5) |
| L1 / L2 | fine lift up / down (ports 9, 10) |
| R1 / R2 | claw close / open |
| A or X | toggle |
| **X + B** held at boot | per-motor diagnostic jog |
| **Y + Down** held at boot | play the stored autonomous routine |

Translation has no deadband; turn and coarse lift use 0.2. Translation and turn
ramp at 10/s and 5/s; the coarse lift is unshaped.

## Motor bindings

| Port(s) | Device |
| --- | --- |
| 1, 2 | Drive front-left (6:1, both reversed) |
| 4, 14 | Drive front-right (6:1) |
| 7, 8 | Drive rear-left (6:1, both reversed) |
| 3, 19 | Drive rear-right (6:1) |
| 9, 10 | Fine lift (18:1, port 10 reversed) |
| 5 | Coarse lift (18:1) |
| 6 | Claw (18:1) |
| 13 | Toggle (18:1) |

Positive wheel RPM means forward for every group. Direction sign-off:
`docs/hardware.md`.

## Autonomous

1. Set `kBootRunMode = RunMode::Record`, build, download.
2. Drive one run. `auton.bin` is written to the SD card on disable.
3. `python tools/ovr_encode.py auton.bin` rewrites `src/auton/programs.cpp`.
4. Set `kBootRunMode = RunMode::Field`, build, download.

The routine replays recorded wheel and mechanism commands on a fixed 20 ms grid,
open loop: it repeats the commands, not the path. Format and limits:
`docs/auton.md`.

## Still needed from the mechanical team

- Wheel diameter and gear tooth counts per wheel group.
- Lift travel limits and safe speeds.
- Mechanism hold/coast behaviour.
- Confirmation that the installed cartridges match 6:1 drive / 18:1 mechanisms.
