# Firmware logic status

## What is implemented

- Portable C++17 control logic for the two Bảng B tasks described in the supplied
  Hướng dẫn lập trình Robot Bảng B slides:
  - line-following from three logical IR inputs;
  - left/right corner handling;
  - first all-black crossing: drive straight for a configurable duration;
  - second all-black crossing: stop and switch to remote mode;
  - remote forward, backward, left, right, and key-off stop.
- Host-side tests that can be run without connecting the robot.

## Important hardware boundary

This is CONTROL LOGIC, not a board-ready firmware image. The slide deck names
E-Robot V1.4 and a Huna mainboard, and describes MAKE/DOWNLOAD in that software.
The user supplied a Huna API header named `functions_codes.h` and a sample program. A board-oriented example is now in `firmware/huna_main.c`. The exact MCU, physical sensor placement, and real sensor polarity still have not been independently verified; no GPIO assignments are needed in this API-level example.

The Huna example currently assumes:
1. `ir(1, 0, NONE) == TRUE` means the left sensor detects black, likewise sensors 2 and 3 for center/right;
2. `rc(UP_KEY/DOWN_KEY/LEFT_KEY/RIGHT_KEY) == TRUE` reports the corresponding remote key;
3. `LMotor_1()` and `RMotor_1()` accept signed speed values as described by the supplied header;
4. motor speed 5 and crossing delay 250 ms are starting values, not measured settings.

Verify sensor placement/polarity and turning direction on the real robot before track use. The separate portable C++ core still uses a board adapter for integration with `robot_logic.hpp`.

The guide says positive motor speed means forward, negative means reverse, and
zero means stop. The core uses this convention. The physical left/right wiring
must be checked on the actual robot.

## Current `main.c` safety updates

The Huna-style `main.c` was updated to:
- give DOWN priority over LEFT/RIGHT in manual mode;
- stop when no remote key is pressed, and require UP for forward motion;
- check the software-estimated 60-second AUTO budget after blocking crossing sampling before issuing another movement command.

These are source-level changes only. The deadline uses accumulated software delay estimates, not a verified hardware clock; long or undocumented blocking calls could make it inaccurate. Confirm behavior and motor polarity on a safely lifted robot, then test on the actual course. The file still relies on undocumented/unverified sensor mapping and Huna API behavior, so it is not certified as competition-ready.

## Recent verification work

- Expanded host tests for line steering, first-crossing timing, second-crossing remote transition, all five remote commands, reset behavior, and unsigned millisecond-counter wrap.
- Added a GitHub Actions workflow at `.github/workflows/host-tests.yml` to configure/build with CMake and run CTest on pushes, pull requests, and manual dispatch.
- The host test executable was compiled locally with `g++ -std=c++17 -Wall -Wextra -Wpedantic` against the portable controller logic, and the test run passed.

This does **not** validate `main.c` against the proprietary Huna environment or prove the physical robot is safe/competition-ready. The Huna SDK/header and board details are not present as buildable local dependencies in this repository view. Treat CI as a portable-logic regression check only.

## Build and test on a development computer

Requirements: CMake 3.16+ and a C++17 compiler.

    cmake -S . -B build
    cmake --build build
    ctest --test-dir build --output-on-failure

These are host-side logic tests only; they do not test motors, sensors, battery
capacity, the E-Robot compiler, or downloading to the mainboard.

## Tune before running

Config defaults are conservative starting values, not measured hardware
settings. Adjust forward_speed, turn_speed, and first_cross_drive_ms on a
safe test track. The all-sensors-black pattern is used by the supplied guide as
the crossing condition; verify the actual sensor placement and polarity first.


## Current Huna two-sensor entry point (2026-11-05)

The repository-root `main.c` now models the user's clarified chassis:
- two rear driven wheels controlled independently by `LMotor_1` and
  `RMotor_1`;
- two passive front wheels;
- two ground-facing IR sensors, read as IR1 and IR3 based on the user's
  earlier sample; IR2 is not read.

The Huna entry point calls `ir(1, 950, SENSE)` and
`ir(3, 950, SENSE)`. It assumes TRUE means black and assumes the sensors
straddle the normal track line: neither sees black when centered, one sees
black during a side deviation, and both see black at a wide crossing. If the
actual sensor placement/polarity differs, the truth table must be adjusted
before the robot is run.

Adaptive behavior in the source:
- centered: straight speed ramps from 3 to 7, one speed step every 50 ms;
- left/right deviation: the inner rear wheel runs at 1 and the outer at 4;
- after a deviation clears: straight motion remains at low speed for 200 ms
  before ramping up;
- both sensors active for three consecutive samples: treated as a crossing;
  first crossing is passed, second crossing stops and enters remote control;
- remote neutral/no key stops both motors;
- remote input now mixes direction and steering: UP+LEFT/RIGHT drives a
  forward arc, DOWN+LEFT/RIGHT drives a reverse arc, LEFT/RIGHT alone pivots
  in place, and UP/DOWN alone drives straight. Conflicting keys cancel the
  corresponding direction/steering axis.

The mixed-key behavior assumes the Huna `rc()` API can report multiple
simultaneously pressed keys. This has not yet been confirmed on the physical
remote/controller. If `rc()` only reports one key at a time, diagonal/arc
commands will not work and the input method must be adapted.

Two binary sensors do not measure an exact curve angle; the code uses a
conservative left/right deviation classification. The speed constants are
starting values and need physical tuning.

**Boot/build status:** the MCU initialization call and Huna API function names
remain from the existing entry point, but this file has NOT been compiled with
the proprietary E-Robot V1.4 toolchain, downloaded to the actual controller,
or boot-tested. A GitHub commit is not proof of successful compilation or boot.
Do not put this version into a live run until it builds in the correct Huna
environment and the sensor truth table and motor directions are checked with
the drive wheels safely lifted. The previous three-sensor PID description is
superseded for the root `main.c`; the portable C++ three-input reference model
and its host tests are unchanged and are not a validation of this Huna file.
