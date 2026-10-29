# Robot Bảng B — NSL 2026–2027

Repository for the Bảng B robot project.

## Confirmed hardware inventory

Only the following hardware is currently confirmed:

- 2 × DC motors
- 2 × servo motors
- 5 × infrared sensors
- 1 × main controller board (exact model not yet identified)

Do not assume extra motors, servos, sensors, motor drivers, batteries, or boards are available. Record any required-but-missing parts separately before purchasing.

## Current design decision

The latest direction is to prioritize a compact line-following robot for a short track of approximately 50 cm, with one right-angle turn and another bend. The user is reconsidering the earlier four-legged design because the controller and motor count are limited.

**This is a provisional direction, not a verified interpretation of the official competition rules.** Check the rules before finalizing the chassis.

A differential-drive chassis with two driven wheels and a caster is a candidate because the two DC motors can independently control left/right motion. The two servos are not required for basic line following and should not be added to the control plan unless they serve a justified purpose.

## Constraints and unknowns

- Exact main board model and available GPIO pins: unknown.
- Whether an H-bridge / DC motor driver is already available: unknown.
- Exact IR sensor module and output type: unknown.
- Track line width, surface, and official course dimensions: not verified.
- Official competition rules and required robot form/mobility: must be checked.
- Battery and power arrangement: not verified.

Do not invent pin assignments, electrical ratings, sensor behavior, dimensions, test results, or competition requirements.

## Development priorities

1. Obtain and document the official rules and scoring conditions.
2. Identify the main board, sensors, motor driver, and power source from labels/photos or documentation.
3. Confirm whether differential-drive line following is allowed.
4. Design the chassis around the confirmed hardware.
5. Implement sensor calibration and line-following logic.
6. Test on a representative track; record results instead of claiming unperformed tests.

## Status

Planning / hardware identification. No claim of a completed or hardware-tested robot.
