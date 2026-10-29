/*
 * BANG B / HUNA E-ROBOT V1.4
 * Adaptive line follower V2 - standalone firmware file.
 *
 * This is Huna C firmware, so it intentionally does NOT include
 * <bits/stdc++.h>: that is a GNU C++ header, not a portable C/Huna header.
 * A separate desktop C++ simulator may use it; the MCU firmware should not.
 *
 * Assumptions requiring bench verification:
 *   IR1 = left, IR2 = center, IR3 = right.
 *   ir(n, 0, NONE) == TRUE means that sensor sees black.
 *   Positive motor speed moves forward.
 *   Motor command values 0..5 are within the Huna API's accepted range.
 *
 * Adaptive speed uses sensor pattern as a curvature proxy:
 *   center only -> faster; center+side -> medium; side only -> slower.
 * No wheel encoders/IMU or sensor geometry are known, so this cannot measure
 * true turn angle, wheel distance, or physical position. Timing is calibrated
 * on the real course, not guessed as an exact distance.
 *
 * The obstacle-pushing bar is fixed; this file does not command a servo for it.
 */
#include "functions_codes.h"

#define SPEED_STRAIGHT       5
#define SPEED_MEDIUM         4
#define SPEED_TIGHT          2
#define CROSS_SPEED          3
#define CROSS_PASS_MS      180
#define CROSS_SAMPLE_MS      5
#define CROSS_STABLE_SAMPLES 3
#define CROSS_MAX_SAMPLES  250
#define LOST_SEARCH_MS      120
#define LOOP_MS               2
#define STAGE_FIRST          0
#define STAGE_SECOND         1

static int last_turn = 0; /* -1 left, +1 right, 0 unknown */

static void stop_drive(void)
{
    LMotor_1(0);
    RMotor_1(0);
}

static unsigned char black(unsigned char n)
{
    return (ir(n, 0, NONE) == TRUE);
}

static void forward_at(int speed)
{
    LMotor_1(speed);
    RMotor_1(speed);
}

static unsigned char all_black(void)
{
    return black(1) && black(2) && black(3);
}

/* Require repeated samples so one noisy reading does not trigger a crossing. */
static unsigned char crossing_stable(void)
{
    unsigned char count = 0;
    unsigned char i;

    for (i = 0; i < CROSS_STABLE_SAMPLES; ++i) {
        if (all_black()) {
            ++count;
        }
        delay(0, 0, 0, CROSS_SAMPLE_MS);
    }
    return (count == CROSS_STABLE_SAMPLES);
}

/* Curve severity is estimated from which sensors see the line, not a true angle. */
static void follow_line_adaptive(void)
{
    unsigned char l = black(1);
    unsigned char c = black(2);
    unsigned char r = black(3);

    if (l && c && r) {
        stop_drive(); /* crossing is handled by the stage machine */
    } else if (l && c) {
        LMotor_1(0);
        RMotor_1(SPEED_MEDIUM);
        last_turn = -1;
    } else if (c && r) {
        LMotor_1(SPEED_MEDIUM);
        RMotor_1(0);
        last_turn = 1;
    } else if (l) {
        /* Sharp left: slow the robot before correcting. */
        LMotor_1(0);
        RMotor_1(SPEED_TIGHT);
        last_turn = -1;
    } else if (r) {
        /* Sharp right: slow the robot before correcting. */
        LMotor_1(SPEED_TIGHT);
        RMotor_1(0);
        last_turn = 1;
    } else if (c) {
        forward_at(SPEED_STRAIGHT);
    } else {
        /* Bounded recovery: do not keep driving blindly if the line is lost. */
        unsigned int elapsed = 0;
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
            delay(0, 0, 0, LOOP_MS);
            elapsed += LOOP_MS;
        }
        stop_drive();
    }
}

/* Move off the first crossing, then verify that the sensors leave it. */
static unsigned char clear_crossing(void)
{
    unsigned int elapsed = 0;

    forward_at(CROSS_SPEED);
    delay(0, 0, 0, CROSS_PASS_MS);
    elapsed += CROSS_PASS_MS;

    while (elapsed < CROSS_MAX_SAMPLES * CROSS_SAMPLE_MS) {
        if (!all_black()) {
            stop_drive();
            delay(0, 0, 0, 40);
            return 1;
        }
        forward_at(CROSS_SPEED);
        delay(0, 0, 0, CROSS_SAMPLE_MS);
        elapsed += CROSS_SAMPLE_MS;
    }

    stop_drive();
    return 0;
}

static void remote_mode(void)
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
        delay(0, 0, 0, LOOP_MS);
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
                if (!clear_crossing()) {
                    /* Fail safe: stop instead of continuing with uncertain position. */
                    for (;;) {
                        stop_drive();
                        delay(0, 0, 0, 100);
                    }
                }
                stage = STAGE_SECOND;
            } else {
                /* Second crossing: finish AUTO and wait for remote commands. */
                stop_drive();
                delay(0, 0, 0, 250);
                remote_mode();
            }
        } else {
            follow_line_adaptive();
        }
        delay(0, 0, 0, LOOP_MS);
    }

    return 0;
}
