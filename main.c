/*
 * BANG B - HUNA E-ROBOT V1.4
 * main.c - standalone adaptive line-following firmware.
 *
 * Functions:
 * - AUTO stage 1: follow black line and pass first crossing.
 * - AUTO stage 2: continue following line to the second crossing.
 * - Then switch to manual remote control.
 * - Adapt speed to the sensor pattern; slow down on tighter turns.
 * - Bounded line-loss recovery and safe stop on unresolved failures.
 * - Assumes the obstacle-pushing bar is fixed mechanically.
 *
 * IMPORTANT HARDWARE ASSUMPTIONS TO VERIFY BEFORE DRIVING:
 * 1. IR1=left, IR2=center, IR3=right.
 * 2. ir(n, 0, NONE)==TRUE means that sensor sees black.
 * 3. Positive motor speed moves forward; motor speed values 0..5 are valid.
 * 4. The first and second all-black patterns really represent crossings on
 *    this course. If a broad black line/marker creates the same pattern,
 *    crossing detection must be calibrated.
 *
 * This is Huna C firmware. Do not include <bits/stdc++.h>: it is a GNU C++
 * header and is not a portable C or Huna firmware header.
 *
 * Without wheel encoders, an IMU, and measured sensor geometry, this program
 * cannot calculate true distance or turn angle. Speed selection below is a
 * heuristic based on the three sensor readings, and all timings need testing
 * on the real robot and course.
 */
#include "functions_codes.h"

#define SPEED_STRAIGHT          5
#define SPEED_MEDIUM            4
#define SPEED_TIGHT              2
#define CROSSING_SPEED           3
#define CROSSING_PASS_MS       180
#define CROSSING_SAMPLE_MS       5
#define CROSSING_STABLE_SAMPLES  3
#define CROSSING_MAX_MS       1250
#define LOST_SEARCH_MS         120
#define LOOP_DELAY_MS            2
#define LEARN_STRAIGHT_TICKS    20
#define SPEED_MIN                2
#define SPEED_MAX                5

#define STAGE_FIRST              0
#define STAGE_SECOND             1

static int last_turn = 0; /* -1 = left, +1 = right, 0 = unknown */

/*
 * Lightweight online adaptation (RAM only, resets on power-up):
 * - Stable center-only readings gradually permit a higher speed.
 * - Turn patterns immediately lower the learned speed.
 * - Line loss drops speed to the safe minimum.
 * This is not route learning and is not saved between runs.
 */
static int learned_speed = SPEED_MEDIUM;
static unsigned int straight_ticks = 0;

static void learn_straight_speed(void)
{
    if (straight_ticks < LEARN_STRAIGHT_TICKS) {
        ++straight_ticks;
    } else if (learned_speed < SPEED_MAX) {
        ++learned_speed;
        straight_ticks = 0;
    }
}

static void learn_turn_speed(int severity)
{
    straight_ticks = 0;
    if (learned_speed > severity) {
        learned_speed = severity;
    }
}

static void learn_line_lost(void)
{
    straight_ticks = 0;
    learned_speed = SPEED_MIN;
}

static void stop_drive(void)
{
    LMotor_1(0);
    RMotor_1(0);
}

static unsigned char black(unsigned char sensor)
{
    return (ir(sensor, 0, NONE) == TRUE);
}

static unsigned char all_black(void)
{
    return (black(1) && black(2) && black(3));
}

static void forward_at(int speed)
{
    LMotor_1(speed);
    RMotor_1(speed);
}

/* Repeated samples reject a single noisy crossing reading. */
static unsigned char crossing_stable(void)
{
    unsigned char count = 0;
    unsigned char i;

    for (i = 0; i < CROSSING_STABLE_SAMPLES; ++i) {
        if (all_black()) {
            ++count;
        }
        delay(0, 0, 0, CROSSING_SAMPLE_MS);
    }
    return (count == CROSSING_STABLE_SAMPLES);
}

/* The sensor pattern is only a rough curvature estimate, not a measured angle. */
static void follow_line_adaptive(void)
{
    unsigned char left = black(1);
    unsigned char center = black(2);
    unsigned char right = black(3);

    if (left && center && right) {
        stop_drive(); /* The stage controller handles crossings. */
    } else if (left && center) {
        learn_turn_speed(SPEED_MEDIUM);
        LMotor_1(0);
        RMotor_1(learned_speed);
        last_turn = -1;
    } else if (center && right) {
        learn_turn_speed(SPEED_MEDIUM);
        LMotor_1(learned_speed);
        RMotor_1(0);
        last_turn = 1;
    } else if (left) {
        /* Sharp left: slow down before correcting. */
        learn_turn_speed(SPEED_TIGHT);
        LMotor_1(0);
        RMotor_1(learned_speed);
        last_turn = -1;
    } else if (right) {
        /* Sharp right: slow down before correcting. */
        learn_turn_speed(SPEED_TIGHT);
        LMotor_1(learned_speed);
        RMotor_1(0);
        last_turn = 1;
    } else if (center) {
        learn_straight_speed();
        forward_at(learned_speed);
    } else {
        /* Search briefly in the last known direction, then stop if still lost. */
        unsigned int elapsed = 0;
        learn_line_lost();

        while (elapsed < LOST_SEARCH_MS) {
            if (black(1) || black(2) || black(3)) {
                return;
            }

            if (last_turn < 0) {
                LMotor_1(0);
                RMotor_1(SPEED_TIGHT);
            } else if (last_turn > 0) {
                LMotor_1(SPEED_TIGHT);
                RMotor_1(0);
            } else {
                stop_drive();
                return;
            }

            delay(0, 0, 0, LOOP_DELAY_MS);
            elapsed += LOOP_DELAY_MS;
        }
        stop_drive();
    }
}

/* Drive off the first crossing; fail stopped if all sensors never clear it. */
static unsigned char clear_first_crossing(void)
{
    unsigned int elapsed = 0;

    forward_at(CROSSING_SPEED);
    delay(0, 0, 0, CROSSING_PASS_MS);
    elapsed += CROSSING_PASS_MS;

    while (elapsed < CROSSING_MAX_MS) {
        if (!all_black()) {
            stop_drive();
            delay(0, 0, 0, 40);
            return 1;
        }

        forward_at(CROSSING_SPEED);
        delay(0, 0, 0, CROSSING_SAMPLE_MS);
        elapsed += CROSSING_SAMPLE_MS;
    }

    stop_drive();
    return 0;
}

static void manual_remote_mode(void)
{
    for (;;) {
        if (rc(UP_KEY) == TRUE) {
            forward_at(SPEED_MEDIUM);
        } else if (rc(DOWN_KEY) == TRUE) {
            LMotor_1(-SPEED_MEDIUM);
            RMotor_1(-SPEED_MEDIUM);
        } else if (rc(LEFT_KEY) == TRUE) {
            LMotor_1(0);
            RMotor_1(SPEED_MEDIUM);
        } else if (rc(RIGHT_KEY) == TRUE) {
            LMotor_1(SPEED_MEDIUM);
            RMotor_1(0);
        } else {
            stop_drive();
        }
        delay(0, 0, 0, LOOP_DELAY_MS);
    }
}

int main(void)
{
    unsigned char stage = STAGE_FIRST;

    mcu_initialization();
    stop_drive();

    for (;;) {
        if (all_black() && crossing_stable()) {
            stop_drive();

            if (stage == STAGE_FIRST) {
                if (!clear_first_crossing()) {
                    /* Safe failure: stop rather than continue with unknown position. */
                    for (;;) {
                        stop_drive();
                        delay(0, 0, 0, 100);
                    }
                }
                stage = STAGE_SECOND;
            } else {
                /* Second crossing ends AUTO and hands control to the remote. */
                stop_drive();
                delay(0, 0, 0, 250);
                manual_remote_mode();
            }
        } else {
            follow_line_adaptive();
        }

        delay(0, 0, 0, LOOP_DELAY_MS);
    }

    return 0;
}
