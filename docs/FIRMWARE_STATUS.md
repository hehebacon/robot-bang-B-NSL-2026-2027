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


## Current Huna three-sensor entry point (2026-11-06)

The repository-root `main.c` now follows the user's confirmed physical layout:
- IR1, IR2, and IR3 are arranged left / center / right in one horizontal row at the front;
- two rear wheels are driven independently; the two front wheels are passive;
- `ir(channel, 950, SENSE)` is assumed to return TRUE on black.

The line controller uses integer-only proportional correction:
- IR1+IR2: gentle left correction;
- IR1 only: stronger left correction;
- IR2 only: straight;
- IR2+IR3: gentle right correction;
- IR3 only: stronger right correction;
- no sensor sees black, or the split IR1+IR3 pattern appears: stop rather than continue with a stale motor command;
- all three sensors see black for four consecutive samples: crossing candidate.

The first confirmed all-three-black marker is crossed at low speed and must clear within a bounded wait; otherwise the firmware remains stopped. The second marker enters the existing latched manual remote loop. The remote loop retains independent rear-motor commands, integer trims, a speed slew limiter, and combined direction/steering targets. Simultaneous key reporting still depends on the Huna `rc()` implementation and is not verified.

**Obstacle limitation:** no forward-facing range/obstacle sensor is installed, so the firmware cannot identify a physical obstacle or perform sensor-based avoidance. The three IR sensors are ground-facing line sensors only. Obstacle/rescue mission actions must remain disabled until a permitted, already-installed detection interface is confirmed; do not infer obstacle presence from line-sensor states.

**Verification boundary:** source-level GitHub changes only. The proprietary Huna E-Robot toolchain/header is not available here, so this version has NOT been compiled with that toolchain, downloaded to the controller, or boot-tested. Confirm sensor order, polarity, threshold, motor direction, and remote-key behavior on a safely secured robot before a track run.

## Manual-mode stability update (2026-11-05)

The repository-root `main.c` now keeps manual mode latched in its infinite
remote-control loop. It has no software time limit or distance limit and does
not intentionally return to the line-following state. Manual motor targets are
limited to magnitude 7, with a two-step steering mix and a one-step-per-30-ms
slew limiter to soften starts, stops, and direction changes.

This is a firmware-side mitigation only. It cannot guarantee that a controller
will not reset when the two DC motors and controller share the same six-cell
AA battery pack. Battery chemistry, cell condition, holder/contact resistance,
motor stall/current draw, board input limits, and the existing power wiring
are not confirmed. A reset can still occur from supply sag or a wiring/contact
fault. Do not treat the slew limiter as proof that the power system is adequate.

The remote input still calls `rc()` once for each key. Simultaneous-key
reporting and the exact behavior of the proprietary Huna API remain
unverified because `functions_codes.h` and a buildable Huna SDK are not
present in this repository. Therefore diagonal/combined-key behavior is not
claimed as fixed.

**Verification boundary:** the GitHub source update is confirmed. No Huna
E-Robot compile, DOWNLOAD, or physical boot was possible from this repository
connection. The portable C++ host tests do not compile or validate this
Huna-specific `main.c`.
