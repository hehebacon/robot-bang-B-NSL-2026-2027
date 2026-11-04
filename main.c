/*
 * BANG B - HUNA E-ROBOT V1.4
 * Three-sensor closed-loop line follower with integer PID control.
 *
 * Uses only the Huna API calls demonstrated in the user's sample:
 *   ir(sensor, 0, NONE), rc(...), LMotor_1, RMotor_1, delay,
 *   mcu_initialization.
 *
 * Sensor convention follows the supplied sample:
 *   ir(n, 0, NONE) == TRUE means sensor n sees the black line.
 *   Sensor 1 = left, 2 = center, 3 = right (verify physical placement).
 *
 * The PID is calculated from the three digital sensor states. It is not
 * analog-position PID. The 5-sensor physical array is NOT assumed here:
 * channels 4/5 and their order have not been confirmed in the supplied API
 * example. Add them only after verifying the Huna API and wiring.
 *
 * This source change does not prove/fix a USB/bootloader transfer stall.
 */
#include "functions_codes.h"

#define BASE_SPEED          4
#define MAX_MOTOR_SPEED    10
#define LOOP_DELAY_MS      10
#define CROSSING_DRIVE_MS 250

/* Integer PID gains; output is in motor-speed units. */
#define KP_NUM              6
#define KD_NUM              2
#define KI_NUM              1
#define PID_SCALE        1000
#define INTEGRAL_LIMIT   1000

#define STAGE_FIRST          0
#define STAGE_SECOND         1

static int previous_error = 0;
static int integral_error = 0;

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
 * Estimate lateral error from the active digital sensors:
 *   left only       -> -1000
 *   left + center   -> -500
 *   center          -> 0
 *   center + right  -> +500
 *   right only      -> +1000
 *
 * The three-sensor all-black pattern is reserved for crossing detection.
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

    if (active_count == 0) {
        return 2000; /* sentinel: line lost */
    }

    return weighted_sum / active_count;
}

/* Calculate a bounded PID correction and mix it into left/right motor speeds. */
static void pid_line_follow(void)
{
    int error = read_line_error();
    int derivative;
    int proportional_term;
    int integral_term;
    int derivative_term;
    int correction;
    int left_speed;
    int right_speed;

    if (error == 2000) {
        /* Do not continue with stale motor commands when the line is lost. */
        motors_stop();
        integral_error = 0;
        previous_error = 0;
        return;
    }

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

    /*
     * Negative error means line is left: slow/reverse left motor and
     * speed up right motor. Positive error does the opposite.
     */
    left_speed = clamp_motor(BASE_SPEED + correction);
    right_speed = clamp_motor(BASE_SPEED - correction);

    LMotor_1(left_speed);
    RMotor_1(right_speed);

    previous_error = error;
}

static void reset_pid(void)
{
    previous_error = 0;
    integral_error = 0;
}

static void remote_control(void)
{
    for (;;) {
        if (rc(UP_KEY) == TRUE) {
            LMotor_1(BASE_SPEED);
            RMotor_1(BASE_SPEED);
        } else if (rc(DOWN_KEY) == TRUE) {
            LMotor_1(-BASE_SPEED);
            RMotor_1(-BASE_SPEED);
        } else if (rc(LEFT_KEY) == TRUE) {
            LMotor_1(-BASE_SPEED);
            RMotor_1(BASE_SPEED);
        } else if (rc(RIGHT_KEY) == TRUE) {
            LMotor_1(BASE_SPEED);
            RMotor_1(-BASE_SPEED);
        } else {
            /* No key (including OFF) means stop. */
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
                /* Pass the first crossing and wait until it is cleared. */
                LMotor_1(BASE_SPEED);
                RMotor_1(BASE_SPEED);
                delay(0, 0, 0, CROSSING_DRIVE_MS);

                while (crossing_detected()) {
                    LMotor_1(BASE_SPEED);
                    RMotor_1(BASE_SPEED);
                    delay(0, 0, 0, LOOP_DELAY_MS);
                }

                motors_stop();
                reset_pid();
                stage = STAGE_SECOND;
            } else {
                /* Second crossing: stop and enter remote-control part. */
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
