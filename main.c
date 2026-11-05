/*
 * BANG B - HUNA E-ROBOT V1.4
 * Three-sensor line follower with integer PID and adaptive speed.
 *
 * Uses only Huna API calls demonstrated in the user's sample.
 * Sensor convention: ir(n, 0, NONE) == TRUE means black line.
 * Sensor 1 = left, 2 = center, 3 = right (verify physical placement).
 *
 * This is a three-digital-sensor controller, not analog-position PID.
 * Channels 4/5 are not used until their API and wiring are verified.
 * This source does not prove/fix a USB/bootloader transfer stall.
 */
#include "functions_codes.h"

#define STRAIGHT_SPEED       7
#define MILD_CURVE_SPEED    5
#define SHARP_CURVE_SPEED   3
#define MAX_MOTOR_SPEED    10
#define LOOP_DELAY_MS      10
#define CROSSING_DRIVE_MS 250
#define CURVE_HOLD_MS     200
#define CURVE_HOLD_TICKS (CURVE_HOLD_MS / LOOP_DELAY_MS)

/* Integer PID gains; initial values still require physical tuning. */
#define KP_NUM              6
#define KD_NUM              2
#define KI_NUM              1
#define PID_SCALE        1000
#define INTEGRAL_LIMIT   1000

#define STAGE_FIRST          0
#define STAGE_SECOND         1

static int previous_error = 0;
static int integral_error = 0;
static int curve_hold_ticks = 0;
static int remembered_curve_speed = STRAIGHT_SPEED;

static void motors_stop(void)
{
    LMotor_1(0);
    RMotor_1(0);
}

static unsigned char sensor_black(unsigned char sensor)
{
    return (ir(sensor, 0, NONE) == TRUE);
}

static unsigned char crossing_detected(void)
{
    return (sensor_black(1) &&
            sensor_black(2) &&
            sensor_black(3));
}

static int clamp_motor(int speed)
{
    if (speed > MAX_MOTOR_SPEED) return MAX_MOTOR_SPEED;
    if (speed < -MAX_MOTOR_SPEED) return -MAX_MOTOR_SPEED;
    return speed;
}

/*
 * Estimate lateral error from three digital sensors:
 * left only=-1000, left+center=-500, center=0,
 * center+right=+500, right only=+1000.
 * No active sensor returns 2000 (line lost).
 */
static int read_line_error(void)
{
    unsigned char left = sensor_black(1);
    unsigned char center = sensor_black(2);
    unsigned char right = sensor_black(3);
    int weighted_sum = 0;
    int active_count = 0;

    if (left) {
        weighted_sum -= 1000;
        ++active_count;
    }
    if (center) {
        ++active_count;
    }
    if (right) {
        weighted_sum += 1000;
        ++active_count;
    }

    if (active_count == 0) return 2000;
    return weighted_sum / active_count;
}

/*
 * Adaptive speed:
 * - center sensor only: straight, accelerate to STRAIGHT_SPEED
 * - adjacent pair: mild curve, use MILD_CURVE_SPEED
 * - outer sensor only: sharp curve, use SHARP_CURVE_SPEED
 * Keep the last curve speed for 200 ms after the line becomes centered,
 * avoiding an immediate speed jump at the curve exit.
 */
static int choose_adaptive_base_speed(int error)
{
    int magnitude = error;

    if (magnitude < 0) magnitude = -magnitude;

    if (error != 0) {
        curve_hold_ticks = CURVE_HOLD_TICKS;

        if (magnitude >= 1000) {
            remembered_curve_speed = SHARP_CURVE_SPEED;
        } else {
            remembered_curve_speed = MILD_CURVE_SPEED;
        }
        return remembered_curve_speed;
    }

    if (curve_hold_ticks > 0) {
        --curve_hold_ticks;
        return remembered_curve_speed;
    }

    remembered_curve_speed = STRAIGHT_SPEED;
    return STRAIGHT_SPEED;
}

/* Calculate bounded PID correction and mix it into left/right motor speeds. */
static void pid_line_follow(void)
{
    int error = read_line_error();
    int base_speed;
    int derivative;
    int proportional_term;
    int integral_term;
    int derivative_term;
    int correction;
    int left_speed;
    int right_speed;

    if (error == 2000) {
        /* Stop instead of driving blindly when all sensors lose the line. */
        motors_stop();
        integral_error = 0;
        previous_error = 0;
        curve_hold_ticks = 0;
        remembered_curve_speed = STRAIGHT_SPEED;
        return;
    }

    base_speed = choose_adaptive_base_speed(error);

    integral_error += error;
    if (integral_error > INTEGRAL_LIMIT) {
        integral_error = INTEGRAL_LIMIT;
    } else if (integral_error < -INTEGRAL_LIMIT) {
        integral_error = -INTEGRAL_LIMIT;
    }

    derivative = error - previous_error;
    proportional_term = (KP_NUM * error) / PID_SCALE;
    integral_term = (KI_NUM * integral_error) / PID_SCALE;
    derivative_term = (KD_NUM * derivative) / PID_SCALE;
    correction = proportional_term + integral_term + derivative_term;

    /* Negative error: line is left. Positive error: line is right. */
    left_speed = clamp_motor(base_speed + correction);
    right_speed = clamp_motor(base_speed - correction);

    LMotor_1(left_speed);
    RMotor_1(right_speed);
    previous_error = error;
}

static void reset_pid(void)
{
    previous_error = 0;
    integral_error = 0;
    curve_hold_ticks = 0;
    remembered_curve_speed = STRAIGHT_SPEED;
}

static void remote_control(void)
{
    for (;;) {
        if (rc(UP_KEY) == TRUE) {
            LMotor_1(STRAIGHT_SPEED);
            RMotor_1(STRAIGHT_SPEED);
        } else if (rc(DOWN_KEY) == TRUE) {
            LMotor_1(-STRAIGHT_SPEED);
            RMotor_1(-STRAIGHT_SPEED);
        } else if (rc(LEFT_KEY) == TRUE) {
            LMotor_1(-STRAIGHT_SPEED);
            RMotor_1(STRAIGHT_SPEED);
        } else if (rc(RIGHT_KEY) == TRUE) {
            LMotor_1(STRAIGHT_SPEED);
            RMotor_1(-STRAIGHT_SPEED);
        } else {
            motors_stop();
        }
        delay(0, 0, 0, LOOP_DELAY_MS);
    }
}

int main(void)
{
    unsigned char stage = STAGE_FIRST;

    mcu_initialization();
    motors_stop();
    reset_pid();

    for (;;) {
        if (crossing_detected()) {
            motors_stop();
            reset_pid();

            if (stage == STAGE_FIRST) {
                LMotor_1(STRAIGHT_SPEED);
                RMotor_1(STRAIGHT_SPEED);
                delay(0, 0, 0, CROSSING_DRIVE_MS);

                while (crossing_detected()) {
                    LMotor_1(STRAIGHT_SPEED);
                    RMotor_1(STRAIGHT_SPEED);
                    delay(0, 0, 0, LOOP_DELAY_MS);
                }

                motors_stop();
                reset_pid();
                stage = STAGE_SECOND;
            } else {
                motors_stop();
                delay(0, 0, 0, 100);
                remote_control();
            }
        } else {
            pid_line_follow();
            delay(0, 0, 0, LOOP_DELAY_MS);
        }
    }

    return 0;
}
