#include "functions_codes.h"

/* Move one integer speed step toward the target. */
int step_toward(int current, int target)
{
    if (current < target)
        return current + 1;
    if (current > target)
        return current - 1;
    return current;
}

int main()
{
    int left_black;
    int center_black;
    int right_black;

    int target_left;
    int target_right;
    int current_left;
    int current_right;
    int ramp_tick;

    int recovery_count;
    int recovery_failed;
    int last_direction;

    int key_up;
    int key_down;
    int key_left;
    int key_right;
    int key_off;
    int manual_active;

    mcu_initialization();

    target_left = 0;
    target_right = 0;
    current_left = 0;
    current_right = 0;
    ramp_tick = 0;
    last_direction = 0;
    manual_active = 0;

    while (1)
    {
        recovery_count = 0;
        recovery_failed = 0;

        /* STAGE 1: autonomous line following. */
        for (;;)
        {
            left_black = (ir(1, 0, NONE) == TRUE);
            center_black = (ir(2, 0, NONE) == TRUE);
            right_black = (ir(3, 0, NONE) == TRUE);

            /* Keep the original all-black stage marker. */
            if (left_black && center_black && right_black)
            {
                target_left = 3;
                target_right = 3;
                current_left = step_toward(current_left, target_left);
                current_right = step_toward(current_right, target_right);
                LMotor_1(current_left);
                RMotor_1(current_right);
                delay(0, 0, 0, 200);
                break;
            }

            if (!left_black && !center_black && !right_black)
            {
                /* Lost line: reverse in bounded, sensor-checked pulses. */
                target_left = -2;
                target_right = -2;
                recovery_count++;

                if (recovery_count > 10)
                {
                    target_left = 0;
                    target_right = 0;
                    recovery_failed = 1;
                }
            }
            else
            {
                recovery_count = 0;

                if (center_black && !left_black && !right_black)
                {
                    target_left = 3;
                    target_right = 3;
                    last_direction = 0;
                }
                else if (left_black && center_black && !right_black)
                {
                    target_left = 2;
                    target_right = 4;
                    last_direction = -1;
                }
                else if (!left_black && center_black && right_black)
                {
                    target_left = 4;
                    target_right = 2;
                    last_direction = 1;
                }
                else if (left_black && !center_black && !right_black)
                {
                    target_left = -2;
                    target_right = 3;
                    last_direction = -1;
                }
                else if (!left_black && !center_black && right_black)
                {
                    target_left = 3;
                    target_right = -2;
                    last_direction = 1;
                }
                else
                {
                    /* Ambiguous pattern: steer toward the last known line side. */
                    if (last_direction < 0)
                    {
                        target_left = -2;
                        target_right = 3;
                    }
                    else if (last_direction > 0)
                    {
                        target_left = 3;
                        target_right = -2;
                    }
                    else
                    {
                        target_left = 2;
                        target_right = 2;
                    }
                }
            }

            /* Same gradual ramp is used for straight motion and turning. */
            ramp_tick++;
            if (ramp_tick >= 2)
            {
                current_left = step_toward(current_left, target_left);
                current_right = step_toward(current_right, target_right);
                ramp_tick = 0;
            }

            LMotor_1(current_left);
            RMotor_1(current_right);
            delay(0, 0, 0, 200);

            if (recovery_failed)
                break;
        }

        if (recovery_failed)
        {
            target_left = 0;
            target_right = 0;
            current_left = 0;
            current_right = 0;
            LMotor_1(0);
            RMotor_1(0);
        }
        else
        {
            /* STAGE 2: second autonomous section. */
            recovery_count = 0;

            for (;;)
            {
                left_black = (ir(1, 0, NONE) == TRUE);
                center_black = (ir(2, 0, NONE) == TRUE);
                right_black = (ir(3, 0, NONE) == TRUE);

                if (left_black && center_black && right_black)
                {
                    target_left = 0;
                    target_right = 0;

                    /* Ramp down instead of abruptly cutting the motors. */
                    while (current_left != 0 || current_right != 0)
                    {
                        current_left = step_toward(current_left, 0);
                        current_right = step_toward(current_right, 0);
                        LMotor_1(current_left);
                        RMotor_1(current_right);
                        delay(0, 0, 0, 200);
                    }
                    break;
                }

                if (!left_black && !center_black && !right_black)
                {
                    target_left = -2;
                    target_right = -2;
                    recovery_count++;

                    if (recovery_count > 10)
                    {
                        recovery_failed = 1;
                        target_left = 0;
                        target_right = 0;
                    }
                }
                else
                {
                    recovery_count = 0;

                    if (center_black && !left_black && !right_black)
                    {
                        target_left = 4;
                        target_right = 4;
                        last_direction = 0;
                    }
                    else if (left_black && center_black && !right_black)
                    {
                        target_left = 2;
                        target_right = 4;
                        last_direction = -1;
                    }
                    else if (!left_black && center_black && right_black)
                    {
                        target_left = 4;
                        target_right = 2;
                        last_direction = 1;
                    }
                    else if (left_black && !center_black && !right_black)
                    {
                        target_left = -2;
                        target_right = 3;
                        last_direction = -1;
                    }
                    else if (!left_black && !center_black && right_black)
                    {
                        target_left = 3;
                        target_right = -2;
                        last_direction = 1;
                    }
                    else
                    {
                        if (last_direction < 0)
                        {
                            target_left = -2;
                            target_right = 3;
                        }
                        else if (last_direction > 0)
                        {
                            target_left = 3;
                            target_right = -2;
                        }
                        else
                        {
                            target_left = 2;
                            target_right = 2;
                        }
                    }
                }

                ramp_tick++;
                if (ramp_tick >= 2)
                {
                    current_left = step_toward(current_left, target_left);
                    current_right = step_toward(current_right, target_right);
                    ramp_tick = 0;
                }

                LMotor_1(current_left);
                RMotor_1(current_right);
                delay(0, 0, 0, 200);

                if (recovery_failed)
                {
                    current_left = 0;
                    current_right = 0;
                    LMotor_1(0);
                    RMotor_1(0);
                    break;
                }
            }
        }

        /*
         * RC mode:
         * Holding a key increases the requested speed gradually.
         * Releasing a moving key gives one short reverse braking pulse,
         * then the motors are stopped and ready for the next command.
         */
        manual_active = 0;

        for (;;)
        {
            key_up = (rc(UP_KEY) == TRUE);
            key_down = (rc(DOWN_KEY) == TRUE);
            key_left = (rc(LEFT_KEY) == TRUE);
            key_right = (rc(RIGHT_KEY) == TRUE);
            key_off = (rc(OFF_KEY) == TRUE);

            if (key_up)
            {
                target_left = 6;
                target_right = 6;
                manual_active = 1;
            }
            else if (key_down)
            {
                target_left = -6;
                target_right = -6;
                manual_active = 1;
            }
            else if (key_left)
            {
                target_left = -6;
                target_right = 6;
                manual_active = 1;
            }
            else if (key_right)
            {
                target_left = 6;
                target_right = -6;
                manual_active = 1;
            }
            else if (key_off)
            {
                target_left = 0;
                target_right = 0;
                manual_active = 0;
            }
            else
            {
                if (manual_active)
                {
                    /* Brief counter-drive to help the robot settle in place. */
                    if (current_left > 0)
                        LMotor_1(-2);
                    else if (current_left < 0)
                        LMotor_1(2);
                    else
                        LMotor_1(0);

                    if (current_right > 0)
                        RMotor_1(-2);
                    else if (current_right < 0)
                        RMotor_1(2);
                    else
                        RMotor_1(0);

                    delay(0, 0, 0, 200);

                    current_left = 0;
                    current_right = 0;
                    target_left = 0;
                    target_right = 0;
                    LMotor_1(0);
                    RMotor_1(0);
                    manual_active = 0;
                }
                else
                {
                    LMotor_1(0);
                    RMotor_1(0);
                    current_left = 0;
                    current_right = 0;
                }

                delay(0, 0, 0, 200);
                continue;
            }

            ramp_tick++;
            if (ramp_tick >= 2)
            {
                current_left = step_toward(current_left, target_left);
                current_right = step_toward(current_right, target_right);
                ramp_tick = 0;
            }

            LMotor_1(current_left);
            RMotor_1(current_right);
            delay(0, 0, 0, 200);
        }
    }

    return 0;
}
