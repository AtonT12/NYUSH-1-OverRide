# NYUSH-1 — VEX U Override Small Robot

VEX V5 C++ competition program for the 2026–2027 VEX U Override Small Robot.

> **Current status:** the C++ program has RPM-based X-drive mixing and the
> reference driver's lift, claw and Toggle bindings. Driver actuator output is
> locked by default until the opposite drivetrain reverse settings in the
> reference code are resolved with a supervised direction check. The secondary
> lift is not mapped, and no autonomous routine is implemented.

## Robot configuration

| Item | Confirmed configuration |
| --- | --- |
| Competition | 2026–2027 VEX U Override |
| Robot category | Small Robot |
| Maximum starting size | 15 × 15 × 15 in |
| Brain | VEX V5 Brain |
| Controller | VEX V5 Controller |
| Robot orientation | Claw side = front; Toggle side = rear |
| Drivetrain | 8-motor Omni X-Drive |
| Drivetrain layout | Four Omni wheels; two motors per wheel through a gear transmission |
| V5 motor cartridges | Drive 6:1; mechanisms 18:1, following the teammate reference |
| Toggle | 1 V5 Smart Motor |
| Primary vertical lift | 2 V5 Smart Motors, one lifting assembly |
| Secondary lift | 1 V5 Smart Motor |
| Claw / Pin gripper | 1 V5 Smart Motor |
| Total motors | 13 |

This is a conventional four-wheel Omni X-drive, **not** a Mecanum drivetrain.
The current per-motor reverse flags come from the existing hardware notes and
normalize positive wheel RPM to the software's forward convention. The teammate
reference uses the opposite reverse flag for every drive motor, so the physical
wheel directions still need a supervised check before driver output is enabled.

The starting-size limit is a mechanical compliance requirement. Software does
not enforce it, and satisfying it does not waive any other VEX U expansion or
robot rule.

## Development environment

- IDE: Visual Studio Code
- Extension: official VEX Robotics VEXcode extension `0.8.2026020401`
- SDK: `V5_20240802_15_00_00`
- Language: C++11 (`-std=gnu++11` from the VEX makefile)
- Project type: VEXcode V5 competition template with the VEX makefile
- Program slot: 1

The project does not use PROS, a custom PID controller, an IMU, odometry, path
following or a third-party library.

### macOS Apple Silicon note

The installed VEX toolchain binaries (`clang`, `make`, `arm-none-eabi-*`) are
x86_64. This arm64 Mac now has Rosetta installed, and the toolchain executes.
On another Apple Silicon build host without Rosetta, install it before using the
VEX build command.

### Verification status

- On 2026-11-16, the RPM driver update built successfully with the bundled VEX
  makefile, Windows toolchain and matching SDK, producing
  `build/NYUSH-1.bin`.
- No program has been downloaded to a physical Brain, and no hardware motion test
  has been performed.

## Directory structure

```text
NYUSH-1/
├── include/
│   ├── vex.h                 # Unchanged VEX template wrapper header
│   ├── constants.h           # Loop period, deadband, RPM setpoints and limits
│   ├── robot-config.h        # Brain, Controller and 13 motor bindings
│   ├── motor-control.h       # Deadband, RPM and null-safe stop helpers
│   ├── drivetrain.h          # Wheel-group and X-drive RPM interface
│   ├── toggle.h              # Toggle mechanism interface
│   ├── vertical-lift.h       # Two-motor primary lift interface
│   ├── secondary-lift.h      # Secondary lift interface
│   ├── claw.h                # Pin gripper interface
│   └── autonomous.h          # Autonomous entry point
├── src/
│   ├── main.cpp              # Competition callbacks and teleop loop
│   ├── robot-config.cpp      # Ports, gear cartridges, directions and readiness
│   ├── motor-control.cpp     # Deadband, RPM limiting, spin/stop helpers
│   ├── drivetrain.cpp        # Four wheel groups and X-drive mixing
│   ├── toggle.cpp            # Toggle implementation
│   ├── vertical-lift.cpp     # Coordinated two-motor lift implementation
│   ├── secondary-lift.cpp    # Secondary lift implementation
│   ├── claw.cpp              # Claw implementation
│   └── autonomous.cpp        # Autonomous strategy entry point
├── vex/                      # Unchanged VEX build rules
├── .vscode/                  # Tracked VEX project file; editor settings ignored
├── makefile                  # Unchanged VEX makefile
└── README.md
```

The VEX makefile automatically includes every `.cpp` directly under `src/`.
No makefile change is needed when these files are added.

## Module responsibilities

| Module | Responsibility |
| --- | --- |
| `robot-config` | Single place for Brain, Controller and all 13 motor bindings; direction normalization; readiness checks; global stop |
| `motor-control` | Shared deadband shaping, RPM limiting, signed RPM command and null-safe stop |
| `drivetrain` | Four logical wheel groups, X-drive mixing, RPM limits and encoder reset |
| `toggle` | Field Toggle RPM command and stop |
| `vertical-lift` | Commands both primary-lift motors together in RPM |
| `secondary-lift` | Articulated secondary-lift RPM command and stop |
| `claw` | Pin gripper RPM command and stop |
| `autonomous` | Strategy entry point for a future verified routine |

`robot-config` does not depend on any mechanism module. Mechanism and drivetrain
modules depend on `robot-config`, and `main.cpp` depends on them. There are no
cyclic includes.

## Current competition behavior

### Pre-autonomous

- Stops every bound motor.
- Resets all eight drivetrain motor encoders to zero after verifying the
  drivetrain binding is complete. This is a position reset only; it does not move
  the robot and assumes the robot is at rest.
- Shows project, hardware-binding and autonomous status on the Brain screen.
- Does not perform fictional IMU, gyro or sensor calibration.

### Autonomous

- Stops every bound motor on entry.
- Calls `Autonomous::run()`.
- Stops every bound motor on exit.
- No route is implemented. Real route work requires verified drivetrain geometry,
  start positions and mechanism travel limits.

### Driver control

- Stops every bound motor on entry.
- Reads forward (`Axis3`), strafe (`Axis4`) and turn (`Axis1`) every 20 ms,
  applies the configured deadband, then computes X-drive wheel RPM.
- Displays the three axis values and whether actuator output is locked.
- When `RobotConstants::kDriverActuationEnabled` is true, L1/L2 raise and lower
  the primary lift, R1/R2 close and open the claw, and A or X raises the Toggle.
- The secondary lift has no driver mapping. The Toggle mapping is one-way, as in
  the reference program.
- The output switch defaults to false. Keep it locked until the drivetrain
  reverse flags have been checked with a supervised low-speed direction test.

The motor RPM setpoints are 500 for translation, 200 for rotation, 20 for the
primary lift, 50 for the claw and 25 for Toggle. Wheel outputs are normalized
together and limited to 480 RPM; mechanism outputs are limited to 120 RPM.

### Stopping behavior

All module stop paths use `vex::brakeType::brake` as the first-version default.
Mechanism-specific `hold`/`coast` behavior remains to be selected after the
mechanical team characterizes each axis.

## Confirmed hardware bindings

All 13 motor objects are declared once in `src/robot-config.cpp` and exposed
through the centralized `RobotHardware` bindings. The reverse flag is applied to
the motor object, not hidden inside a mechanism module.

| Logical motor | Binding | Port | Reported native positive | Reverse flag | Logical positive |
| --- | --- | --- | --- | --- | --- |
| Drive FL A | `driveFrontLeft.motorA` | 1 | backward | Yes | wheel forward |
| Drive FL B | `driveFrontLeft.motorB` | 2 | backward | Yes | wheel forward |
| Drive FR A | `driveFrontRight.motorA` | 4 | forward | No | wheel forward |
| Drive FR B | `driveFrontRight.motorB` | 14 | forward | No | wheel forward |
| Drive RL A | `driveRearLeft.motorA` | 7 | backward | Yes | wheel forward |
| Drive RL B | `driveRearLeft.motorB` | 8 | backward | Yes | wheel forward |
| Drive RR A | `driveRearRight.motorA` | 3 | forward | No | wheel forward |
| Drive RR B | `driveRearRight.motorB` | 19 | forward | No | wheel forward |
| Toggle | `toggleMotor` | 13 | upward | No | upward |
| Primary lift A | `primaryLift.motorA` | 9 | raise | No | raise |
| Primary lift B | `primaryLift.motorB` | 10 | lower | Yes | raise |
| Secondary lift | `secondaryLiftMotor` | 5 | raise | No | raise |
| Claw | `clawMotor` | 6 | close | No | close |

Direction and cartridge notes:

- The current drivetrain flags are FL/RL reversed and FR/RR not reversed, based
  on the existing hardware notes. The teammate reference uses the opposite
  flags on all four wheel groups. Keep driver output locked until the physical
  wheel directions have been checked.
- Primary Lift B is reverse-normalized because its native positive direction
  was reported to lower the assembly. Positive VerticalLift RPM therefore
  raises both sides together according to the existing hardware notes.
- Toggle, Secondary Lift and Claw already have native positive directions that
  match their module conventions according to the existing hardware notes.
- The code sets the drive motors to the 6:1 cartridge and all mechanism motors
  to 18:1 based on the teammate reference. Verify that the physical cartridges
  match before using RPM values on the robot.
- Motor commands use RPM at the motor output shaft. The external drivetrain
  transmission ratio and wheel diameter are not encoded, so RPM is not a
  chassis-distance or linear-speed conversion.
- Readiness is checked against the hardware, not just the source: every
  subsystem requires its motor pointers to be bound *and* each motor to report
  `installed()`. An unplugged motor therefore marks its subsystem not ready.
- A not-ready command path stops its motors before returning, so a cable fault
  cannot leave the surviving motors at their last command.
- The modules still require complete two-motor pairs. A half-bound pair refuses
  to move, while `stopAllMotors()` can still stop the motor that is present.

The software wheel signs define a forward/strafe/turn convention, but the
physical wheel response has not been tested. Keep `kDriverActuationEnabled`
false until a supervised low-speed direction check confirms the chassis motion.

## Hardware information still required

- Gear tooth counts and gear ratio for each wheel group.
- Wheel diameter and exact wheel mounting/axis convention for X-drive mixing.
- Allowed lift travel/angle and safe speed limits.
- Mechanism stop/hold requirements.
- Secondary-lift operator mapping.
- Autonomous starting positions and strategy.

## Build and download

### From VS Code

1. Open this folder in VS Code with the VEX extension installed.
   `.vscode/vex_project_settings.json` is tracked, so the folder opens as a VEX
   project with the project name, program slot 1 and SDK version already set.
   The remaining `.vscode` files are editor-local and git-ignored.
2. Open the VEX sidebar and use **Build**.
3. Review all compiler and linker output before downloading.
4. Download only after the hardware configuration is reviewed and the Brain is
   physically safe to test.
5. Download to program slot 1 as configured in `.vscode/vex_project_settings.json`.

### From a terminal (same VEX makefile)

```bash
TOOLCHAIN="$HOME/Library/Application Support/Code/User/globalStorage/vexrobotics.vexcode/tools/cpp/toolchain_osxarm64"
SDK_HOME="$HOME/Library/Application Support/Code/User/globalStorage/vexrobotics.vexcode/sdk/cpp/V5/V5_20240802_15_00_00"

PATH="$TOOLCHAIN/gcc/bin:$TOOLCHAIN/tools/bin:$TOOLCHAIN/clang/bin:$PATH" \
  make T="$SDK_HOME" P=NYUSH-1
```

This command is the project's own VEX build flow, not a generic host build. The
current arm64 Mac has Rosetta installed and completed this build successfully.
The generated `build/` directory is ignored by `.gitignore`.

Do not download or run the program on a physical Brain without explicit user
authorization and a safe mechanical test setup.

## Next development steps

1. Verify that the physical motor cartridges match the configured 6:1 drive and
   18:1 mechanism settings.
2. Resolve the differing drivetrain reverse flags with a supervised low-speed
   wheel-direction check while `kDriverActuationEnabled` remains false by default.
3. Enable driver output only after direction and mechanism travel checks, then
   tune RPM setpoints and stopping behavior on the robot.
4. Add the secondary-lift mapping and autonomous routine after their safe limits
   and operating behavior are established.
