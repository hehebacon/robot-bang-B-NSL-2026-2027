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

Do not assume extra motors, servos, sensors, motor drivers, batteries, or boards are available. Record any required-but-missing parts separately before purchasing.

## Current design direction

Prioritize a compact line-following robot. A differential-drive chassis with two driven wheels and a caster is a candidate because the two DC motors can independently control left/right motion. Keep the full design within the 250 mm cube.

The two servos are not required for basic line following; only include them if a useful, rule-compliant purpose is established. The earlier four-legged robot concept is not the default design under the currently confirmed hardware limits.

**This is provisional:** confirm the official rules and required robot form/mobility before finalizing the chassis or control strategy.

## Constraints and unknowns

- Exact main board model and available GPIO pins: unknown.
- Whether an H-bridge / DC motor driver is already available: unknown.
- Exact IR sensor module and output type: unknown.
- Track line width, surface, and exact course layout: not verified.
- Official competition rules and scoring conditions: must be checked.
- Battery and power arrangement: not verified.

Do not invent pin assignments, electrical ratings, sensor behavior, dimensions, test results, or competition requirements.

## Development priorities

1. Obtain and document the official rules and scoring conditions.
2. Identify the main board, sensors, motor driver, and power source from labels/photos or documentation without modifying the circuits.
3. Confirm whether differential-drive line following is allowed.
4. Design the exterior/chassis within 250 × 250 × 250 mm around the confirmed hardware.
5. Implement sensor calibration and line-following logic only within the permitted rules and existing electronics.
6. Test on a representative track and record actual results; do not claim tests that have not been performed.

## Status

Planning / hardware identification. No claim of a completed or hardware-tested robot.
