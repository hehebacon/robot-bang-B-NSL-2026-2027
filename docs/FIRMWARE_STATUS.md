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


## Current Huna PID entry point (2026-11-04)

The repository root `main.c` has been simplified to an integer-only PID-style
line follower using the three digital IR channels demonstrated by the user's
working sample. It calculates a weighted lateral error from left/center/right
sensor states and combines proportional, integral, and derivative terms to
adjust the two motor speeds. It avoids floating-point arithmetic and standard
library dependencies.

The current code assumes:
- IR1/IR2/IR3 are physically left/center/right;
- `ir(n, 0, NONE) == TRUE` indicates black, as in the supplied sample;
- `LMotor_1` and `RMotor_1` accept signed speed commands in the range used
  by the sample;
- positive/negative motor directions correspond to the assumed left/right
  wheel orientation.

The robot is reported to have five physical IR sensors, but the supplied sample
only demonstrates channels 1–3. The code does not call channels 4 or 5 until
their numbering, placement, and API behavior are confirmed. Therefore this is
a three-channel digital PID approximation, not five-sensor analog PID.

The first all-black event passes the crossing and waits until the three
channels no longer all report black before resuming line following. The second
all-black event stops and enters remote control. This prevents the same
wide crossing from immediately being counted twice.

**Not yet verified:** compilation in Huna E-Robot V1.4, Download to the actual
mainboard, physical motor polarity, sensor order/polarity, PID gains, crossing
timing, and the reported 50% transfer stall. A successful source commit is not
evidence that the E-Robot transfer problem is fixed. Test with the drive wheels
safely lifted before placing the robot on the course.
