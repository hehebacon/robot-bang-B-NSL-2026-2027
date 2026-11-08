#include "functions_codes.h"

/*
 * Automatic line follower:
 * - Accelerates gradually on a centered straight section.
 * - Slows down for curves.
 * - Uses a stronger steering command for outer-sensor-only sharp turns.
 * Sensor polarity and steering directions follow the user's known-working code.
 *
 * Acceleration is based on loop counts, not calibrated seconds. Tune only
 * after testing on the actual track.
 */
#define AUTO_START_SPEED       3
#define AUTO_MAX_SPEED         5
#define AUTO_ACCEL_TICKS       300
#define CURVE_TURN_SPEED       4
#define SHARP_TURN_SPEED       6

static int auto_speed = AUTO_START_SPEED;
static unsigned int straight_ticks = 0;

/* Returns 1 when the all-black marker is reached, otherwise 0. */
static int auto_line_step(int stop_at_marker)
{
    unsigned char l_black = (ir(1,0,NONE)==TRUE);
    unsigned char c_black = (ir(2,0,NONE)==TRUE);
    unsigned char r_black = (ir(3,0,NONE)==TRUE);

    /* All three sensors see black: stage marker. */
    if(l_black && c_black && r_black)
    {
        straight_ticks = 0;
        if(stop_at_marker)
        {
            LMotor_1(0);
            RMotor_1(0);
        }
        else
        {
            LMotor_1(auto_speed);
            RMotor_1(auto_speed);
            delay(0,0,0,250);
        }
        return 1;
    }

    /* Center sensor sees the line: accelerate progressively on straight. */
    if(!l_black && c_black && !r_black)
    {
        straight_ticks++;
        if(straight_ticks >= AUTO_ACCEL_TICKS)
        {
            straight_ticks = 0;
            if(auto_speed < AUTO_MAX_SPEED)
                auto_speed++;
        }

        LMotor_1(auto_speed);
        RMotor_1(auto_speed);
        return 0;
    }

    /* Line is toward the left: slow for a curve, stronger for a sharp turn. */
    if(l_black && c_black && !r_black)
    {
        straight_ticks = 0;
        auto_speed = AUTO_START_SPEED;
        LMotor_1(-CURVE_TURN_SPEED);
        RMotor_1(CURVE_TURN_SPEED);
        return 0;
    }

    if(l_black && !c_black && !r_black)
    {
        straight_ticks = 0;
        auto_speed = AUTO_START_SPEED;
        LMotor_1(-SHARP_TURN_SPEED);
        RMotor_1(SHARP_TURN_SPEED);
        return 0;
    }

    /* Line is toward the right. */
    if(!l_black && c_black && r_black)
    {
        straight_ticks = 0;
        auto_speed = AUTO_START_SPEED;
        LMotor_1(CURVE_TURN_SPEED);
        RMotor_1(-CURVE_TURN_SPEED);
        return 0;
    }

    if(!l_black && !c_black && r_black)
    {
        straight_ticks = 0;
        auto_speed = AUTO_START_SPEED;
        LMotor_1(SHARP_TURN_SPEED);
        RMotor_1(-SHARP_TURN_SPEED);
        return 0;
    }

    /*
     * Unrecognized sensor pattern: stop instead of keeping an old steering
     * command active and accidentally driving in circles.
     */
    straight_ticks = 0;
    LMotor_1(0);
    RMotor_1(0);
    return 0;
}

int main ()
{
    mcu_initialization();

    while(1)
    {
        /* Stage 1: follow line and pass the first all-black marker. */
        for(;;)
        {
            if(auto_line_step(0)==1)
                break;
        }

        /* Stage 2: follow line and stop at the next all-black marker. */
        for(;;)
        {
            if(auto_line_step(1)==1)
                break;
        }

        /* Manual RC mode retained from the supplied working code. */
        for(;;)
        {
            if(rc(UP_KEY)==TRUE)
            {
                LMotor_1(10);
                RMotor_1(10);
            }
            if(rc(DOWN_KEY)==TRUE)
            {
                LMotor_1(-10);
                RMotor_1(-10);
            }
            if(rc(LEFT_KEY)==TRUE)
            {
                LMotor_1(-10);
                RMotor_1(10);
            }
            if(rc(RIGHT_KEY)==TRUE)
            {
                LMotor_1(10);
                RMotor_1(-10);
            }
            if(rc(OFF_KEY)==TRUE)
            {
                LMotor_1(0);
                RMotor_1(0);
            }
        }
    }

    return 0;
}
