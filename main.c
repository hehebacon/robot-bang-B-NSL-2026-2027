/*
 * BANG B - HUNA E-ROBOT V1.4
 * Two-IR differential-drive line follower with gradual speed control.
 *
 * Hardware model supplied by the user:
 * - rear left/right wheels are driven by LMotor_1 / RMotor_1;
 * - front wheels are passive;
 * - two ground-facing IR sensors are used.
 *
 * Sensor channels follow the user's earlier sample:
 *   IR1 = left, IR3 = right; IR2 is not read.
 *   ir(channel, 950, SENSE) == TRUE is assumed to mean black line.
 *
 * IMPORTANT SENSOR-GEOMETRY ASSUMPTION:
 * The two sensors must straddle the normal line: neither sees black when
 * centered, one sees black when the robot drifts, and both see black at a
 * wide crossing. Confirm this on the actual chassis before track use.
 *
 * This file uses only the Huna API names already shown in the user's sample.
 * It does not prove compatibility with the proprietary E-Robot compiler,
 * successful DOWNLOAD, or successful boot on the physical board.
 */
#include "functions_codes.h"

#define LOOP_DELAY_MS       10
#define CROSSING_DRIVE_MS  250
#define CROSSING_CONFIRM_COUNT 3

#define STRAIGHT_MIN_SPEED  3
#define STRAIGHT_MAX_SPEED  7
#define CURVE_INNER_SPEED   1
#define CURVE_OUTER_SPEED   4
#define SPEED_RAMP_INTERVAL_MS 50
#define CURVE_RECOVERY_MS  200
#define CURVE_RECOVERY_TICKS (CURVE_RECOVERY_MS / LOOP_DELAY_MS)

#define STAGE_FIRST  0
#define STAGE_SECOND 1

static int straight_speed = STRAIGHT_MIN_SPEED;
static int ramp_elapsed_ms = 0;
static int curve_recovery_ticks = 0;

static void motors_stop(void)
{
    LMotor_1(0);
    RMotor_1(0);
}

static unsigned char left_on_black(void)
{
    return (ir(1, 950, SENSE) == TRUE);
}

static unsigned char right_on_black(void)
{
    return (ir(3, 950, SENSE) == TRUE);
}

static void reset_speed_state(void)
{
    straight_speed = STRAIGHT_MIN_SPEED;
    ramp_elapsed_ms = 0;
    curve_recovery_ticks = 0;
}

static void drive_straight_gradually(void)
{
    if (curve_recovery_ticks > 0) {
        --curve_recovery_ticks;
        LMotor_1(STRAIGHT_MIN_SPEED);
        RMotor_1(STRAIGHT_MIN_SPEED);
        return;
    }

    ramp_elapsed_ms += LOOP_DELAY_MS;
    if (ramp_elapsed_ms >= SPEED_RAMP_INTERVAL_MS) {
        ramp_elapsed_ms = 0;
        if (straight_speed < STRAIGHT_MAX_SPEED) {
            ++straight_speed;
        }
    }

    LMotor_1(straight_speed);
    RMotor_1(straight_speed);
}

/*
 * With sensors straddling the line:
 * - neither sees black: centered, accelerate gradually;
 * - left sees black: line is under left sensor, steer left;
 * - right sees black: line is under right sensor, steer right;
 * - both see black: handled by crossing state machine.
 *
 * Binary sensors cannot measure an exact corner angle. A detected side
 * deviation is treated conservatively as a corner and both wheels slow down.
 */
static void follow_line(unsigned char left_black,
                        unsigned char right_black)
{
    if (!left_black && !right_black) {
        drive_straight_gradually();
    } else if (left_black && !right_black) {
        straight_speed = STRAIGHT_MIN_SPEED;
        ramp_elapsed_ms = 0;
        curve_recovery_ticks = CURVE_RECOVERY_TICKS;
        LMotor_1(CURVE_INNER_SPEED);
        RMotor_1(CURVE_OUTER_SPEED);
    } else if (!left_black && right_black) {
        straight_speed = STRAIGHT_MIN_SPEED;
        ramp_elapsed_ms = 0;
        curve_recovery_ticks = CURVE_RECOVERY_TICKS;
        LMotor_1(CURVE_OUTER_SPEED);
        RMotor_1(CURVE_INNER_SPEED);
    } else {
        motors_stop();
    }
}

#define REMOTE_TURN_DELTA 3
#define REMOTE_MAX_SPEED 10

static int clamp_remote_speed(int speed)
{
    if (speed > REMOTE_MAX_SPEED) return REMOTE_MAX_SPEED;
    if (speed < -REMOTE_MAX_SPEED) return -REMOTE_MAX_SPEED;
    return speed;
}

/*
 * Remote control supports combined direction + steering:
 * UP + LEFT/RIGHT = forward arc; DOWN + LEFT/RIGHT = reverse arc.
 * LEFT/RIGHT alone pivots in place; UP/DOWN alone drives straight.
 * This reads each key independently, so simultaneous-key reporting by
 * the remote/controller must be supported by the Huna rc() API.
 */
static void remote_control(void)
{
    for (;;) {
        unsigned char up = (rc(UP_KEY) == TRUE);
        unsigned char down = (rc(DOWN_KEY) == TRUE);
        unsigned char left = (rc(LEFT_KEY) == TRUE);
        unsigned char right = (rc(RIGHT_KEY) == TRUE);
        int base_speed = 0;
        int turn_delta = 0;
        int left_speed;
        int right_speed;

        if (up && !down) {
            base_speed = STRAIGHT_MAX_SPEED;
        } else if (down && !up) {
            base_speed = -STRAIGHT_MAX_SPEED;
        }

        if (left && !right) {
            turn_delta = REMOTE_TURN_DELTA;
        } else if (right && !left) {
            turn_delta = -REMOTE_TURN_DELTA;
        }

        if (base_speed == 0 && turn_delta == 0) {
            motors_stop();
        } else {
            left_speed = clamp_remote_speed(base_speed - turn_delta);
            right_speed = clamp_remote_speed(base_speed + turn_delta);
            LMotor_1(left_speed);
            RMotor_1(right_speed);
        }

        delay(0, 0, 0, LOOP_DELAY_MS);
    }
}

int main(void)
{
    unsigned char stage = STAGE_FIRST;
    unsigned char crossing_count = 0;

    mcu_initialization();
    motors_stop();
    reset_speed_state();

    for (;;) {
        unsigned char left_black = left_on_black();
        unsigned char right_black = right_on_black();

        /* Debounce the two-sensor wide-black crossing pattern. */
        if (left_black && right_black) {
            if (crossing_count < CROSSING_CONFIRM_COUNT) {
                ++crossing_count;
            }
        } else {
            crossing_count = 0;
        }

        if (crossing_count >= CROSSING_CONFIRM_COUNT) {
            motors_stop();
            reset_speed_state();

            if (stage == STAGE_FIRST) {
                /* Pass the first crossing, then wait for both sensors to clear. */
                LMotor_1(STRAIGHT_MIN_SPEED);
                RMotor_1(STRAIGHT_MIN_SPEED);
                delay(0, 0, 0, CROSSING_DRIVE_MS);

                while (left_on_black() && right_on_black()) {
                    LMotor_1(STRAIGHT_MIN_SPEED);
                    RMotor_1(STRAIGHT_MIN_SPEED);
                    delay(0, 0, 0, LOOP_DELAY_MS);
                }

                motors_stop();
                reset_speed_state();
                crossing_count = 0;
                stage = STAGE_SECOND;
            } else {
                motors_stop();
                delay(0, 0, 0, 100);
                remote_control();
            }
        } else {
            follow_line(left_black, right_black);
            delay(0, 0, 0, LOOP_DELAY_MS);
        }
    }

    return 0;
}
