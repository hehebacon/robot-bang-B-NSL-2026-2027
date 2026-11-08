#include "functions_codes.h"

/*
 * 3-sensor line follower:
 * IR 1 = LEFT, IR 2 = CENTER, IR 3 = RIGHT.
 * Black: ir(channel, 0, NONE) == TRUE
 * White/non-black: ir(channel, 950, SENSE) == TRUE
 *
 * If all sensors lose the black line, reverse in short pulses.
 * After 10 unsuccessful pulses, stop instead of reversing indefinitely.
 */

int main()
{
    int left_black;
    int center_black;
    int right_black;
    int recovery_count;
    int recovery_failed;
    int last_direction;

    mcu_initialization();

    last_direction = 0;

    while (1)
    {
        recovery_count = 0;
        recovery_failed = 0;

        /* STAGE 1 */
        for (;;)
        {
            left_black = (ir(1, 0, NONE) == TRUE);
            center_black = (ir(2, 0, NONE) == TRUE);
            right_black = (ir(3, 0, NONE) == TRUE);

            /* Original stage marker: all three sensors see black. */
            if (left_black && center_black && right_black)
            {
                LMotor_1(3);
                RMotor_1(3);
                delay(0, 0, 0, 200);
                break;
            }

            /* Lost line: reverse slowly, checking again after each pulse. */
            if (!left_black && !center_black && !right_black)
            {
                if (recovery_count < 10)
                {
                    LMotor_1(-2);
                    RMotor_1(-2);
                    delay(0, 0, 0, 200);
                    recovery_count++;
                }
                else
                {
                    LMotor_1(0);
                    RMotor_1(0);
                    recovery_failed = 1;
                    break;
                }
            }
            else
            {
                recovery_count = 0;

                /* Follow smoothly: slow the inner wheel on curves. */
                if (center_black && !left_black && !right_black)
                {
                    LMotor_1(3);
                    RMotor_1(3);
                    last_direction = 0;
                }
                else if (left_black && center_black && !right_black)
                {
                    LMotor_1(2);
                    RMotor_1(4);
                    last_direction = -1;
                }
                else if (!left_black && center_black && right_black)
                {
                    LMotor_1(4);
                    RMotor_1(2);
                    last_direction = 1;
                }
                else if (left_black && !center_black && !right_black)
                {
                    LMotor_1(-2);
                    RMotor_1(3);
                    last_direction = -1;
                }
                else if (!left_black && !center_black && right_black)
                {
                    LMotor_1(3);
                    RMotor_1(-2);
                    last_direction = 1;
                }
                else if (left_black && center_black && right_black == 0)
                {
                    LMotor_1(2);
                    RMotor_1(4);
                    last_direction = -1;
                }
                else if (left_black == 0 && center_black && right_black)
                {
                    LMotor_1(4);
                    RMotor_1(2);
                    last_direction = 1;
                }
                else
                {
                    /* Ambiguous sensor pattern: use the last known line side. */
                    if (last_direction < 0)
                    {
                        LMotor_1(-2);
                        RMotor_1(3);
                    }
                    else if (last_direction > 0)
                    {
                        LMotor_1(3);
                        RMotor_1(-2);
                    }
                    else
                    {
                        LMotor_1(2);
                        RMotor_1(2);
                    }
                }
            }
        }

        if (recovery_failed)
        {
            LMotor_1(0);
            RMotor_1(0);
        }
        else
        {
            /* STAGE 2 */
            recovery_count = 0;

            for (;;)
            {
                left_black = (ir(1, 0, NONE) == TRUE);
                center_black = (ir(2, 0, NONE) == TRUE);
                right_black = (ir(3, 0, NONE) == TRUE);

                /* Original stage-2 marker: all three sensors see black. */
                if (left_black && center_black && right_black)
                {
                    LMotor_1(0);
                    RMotor_1(0);
                    break;
                }

                if (!left_black && !center_black && !right_black)
                {
                    if (recovery_count < 10)
                    {
                        LMotor_1(-2);
                        RMotor_1(-2);
                        delay(0, 0, 0, 200);
                        recovery_count++;
                    }
                    else
                    {
                        LMotor_1(0);
                        RMotor_1(0);
                        recovery_failed = 1;
                        break;
                    }
                }
                else
                {
                    recovery_count = 0;

                    if (center_black && !left_black && !right_black)
                    {
                        LMotor_1(4);
                        RMotor_1(4);
                        last_direction = 0;
                    }
                    else if (left_black && center_black && !right_black)
                    {
                        LMotor_1(2);
                        RMotor_1(4);
                        last_direction = -1;
                    }
                    else if (!left_black && center_black && right_black)
                    {
                        LMotor_1(4);
                        RMotor_1(2);
                        last_direction = 1;
                    }
                    else if (left_black && !center_black && !right_black)
                    {
                        LMotor_1(-2);
                        RMotor_1(3);
                        last_direction = -1;
                    }
                    else if (!left_black && !center_black && right_black)
                    {
                        LMotor_1(3);
                        RMotor_1(-2);
                        last_direction = 1;
                    }
                    else if (left_black && center_black && right_black == 0)
                    {
                        LMotor_1(2);
                        RMotor_1(4);
                        last_direction = -1;
                    }
                    else if (left_black == 0 && center_black && right_black)
                    {
                        LMotor_1(4);
                        RMotor_1(2);
                        last_direction = 1;
                    }
                    else
                    {
                        if (last_direction < 0)
                        {
                            LMotor_1(-2);
                            RMotor_1(3);
                        }
                        else if (last_direction > 0)
                        {
                            LMotor_1(3);
                            RMotor_1(-2);
                        }
                        else
                        {
                            LMotor_1(2);
                            RMotor_1(2);
                        }
                    }
                }
            }
        }

        /* RC control remains active after the autonomous stages. */
        for (;;)
        {
            if (rc(UP_KEY) == TRUE)
            {
                LMotor_1(6);
                RMotor_1(6);
            }
            else if (rc(DOWN_KEY) == TRUE)
            {
                LMotor_1(-6);
                RMotor_1(-6);
            }
            else if (rc(LEFT_KEY) == TRUE)
            {
                LMotor_1(-6);
                RMotor_1(6);
            }
            else if (rc(RIGHT_KEY) == TRUE)
            {
                LMotor_1(6);
                RMotor_1(-6);
            }
            else if (rc(OFF_KEY) == TRUE)
            {
                LMotor_1(0);
                RMotor_1(0);
            }
        }
    }

    return 0;
}
