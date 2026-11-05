# Robot Bảng B — NSL 2026–2027

Repository for the Bảng B robot project.

## Confirmed constraints

- **Maximum outer dimensions:** 250 × 250 × 250 mm (25 × 25 × 25 cm). Design and verify the complete assembled robot against all three axes, including wheels, sensors, covers, and decorative parts.
- **Appearance/mechanical changes are allowed:** chassis, shell, shape, mounting brackets, and exterior styling may be redesigned.
- **Circuit changes are not allowed:** preserve the existing main board and electronic circuits. Do not modify circuit boards or make unapproved electronic circuit changes. Any firmware/configuration changes must still comply with the official competition rules.
- **Track description so far:** approximately 50 cm long, with one right-angle turn and another bend. Official track details still need confirmation.

## Confirmed hardware inventory

Only the following hardware is currently confirmed:

- 2 × DC motors
- 2 × servo motors
- 5 × infrared sensors
- 1 × main controller board (exact model not yet identified)
- 1 × 9 V battery (battery type/chemistry and condition not yet confirmed)

Do not assume extra motors, servos, sensors, motor drivers, batteries, or boards are available. Record any required-but-missing parts separately before purchasing.

## Current design direction

Prioritize a compact line-following robot. A differential-drive chassis with two driven wheels and a caster is a candidate because the two DC motors can independently control left/right motion. Keep the full design within the 250 mm cube.

The two servos are not required for basic line following; only include them if a useful, rule-compliant purpose is established. The earlier four-legged robot concept is not the default design under the currently confirmed hardware limits.

**This is provisional:** confirm the official rules and required robot form/mobility before finalizing the chassis or control strategy.

## Firmware logic (new)

- Portable C++17 controller core: [firmware/robot_logic.hpp](firmware/robot_logic.hpp)
- Host-side tests: [tests/robot_logic_test.cpp](tests/robot_logic_test.cpp)
- Build and test: [CMakeLists.txt](CMakeLists.txt)
- Current Huna / E-Robot two-sensor entry point using `functions_codes.h`: [main.c](main.c)
- Older three-sensor Huna reference (not the current two-sensor target): [firmware/huna_main.c](firmware/huna_main.c)
- Hardware integration notes: [docs/FIRMWARE_STATUS.md](docs/FIRMWARE_STATUS.md)

The portable core and host tests remain a separate three-input reference model. The Huna entry point in [main.c](main.c) has been changed to a **two-IR differential-drive** controller: it reads IR1 and IR3, drives the two rear wheels independently, ramps straight-line speed gradually, slows both wheels for a detected side deviation, and keeps a 200 ms low-speed recovery period after a curve. The two front wheels are treated as passive.

**Important assumptions and limits:** `main.c` follows the user's earlier sample by calling `ir(1, 950, SENSE)` and `ir(3, 950, SENSE)`; IR1 is assumed left, IR3 right, and TRUE is assumed to mean black. It assumes the sensors straddle the normal line: neither sees black when centered, one sees black on a deviation, and both see black at a wide crossing. Confirm this sensor geometry and polarity on the real chassis before use. Two binary sensors cannot calculate an exact corner angle. This source change has not been compiled with the proprietary Huna E-Robot V1.4 toolchain, downloaded to the board, or boot-tested. The portable C++ tests do not validate this Huna entry point.

To run the host tests with CMake 3.16+ and a C++17 compiler:

    cmake -S . -B build
    cmake --build build
    ctest --test-dir build --output-on-failure

## Constraints and unknowns

- Exact main board model and available GPIO pins: unknown.
- Whether an H-bridge / DC motor driver is already available: unknown.
- Exact IR sensor module and output type: unknown.
- Track line width, surface, and exact course layout: not verified.
- Official competition rules and scoring conditions: must be checked.
- The user reports a 9 V battery is available, but its type/chemistry, condition, connector, and usable current are not verified.
- Power compatibility is not established: a common rectangular 9 V battery may be unable to supply the peak current required by DC motors and servos. Check the board/motor specifications and power requirements before connecting it; do not connect the battery directly to unknown pins or assume it can safely power every component.

Do not invent pin assignments, electrical ratings, sensor behavior, dimensions, test results, or competition requirements.

## Development priorities

1. Obtain and document the official rules and scoring conditions.
2. Identify the main board, sensors, motor driver, and 9 V battery type from labels/photos or documentation without modifying the circuits; verify current and voltage requirements before powering the robot.
3. Confirm whether differential-drive line following is allowed.
4. Design the exterior/chassis within 250 × 250 × 250 mm around the confirmed hardware.
5. Implement a board-specific adapter only after the exact controller and existing interfaces are identified.
6. Test on a representative track and record actual results; do not claim tests that have not been performed.

## Status

Portable C++ control logic committed; board integration and physical testing remain pending.
