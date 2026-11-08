#include "functions_codes.h"

int main ()
{
    mcu_initialization();

    while(1)
    {
        /* Stage 1: proven line-following behavior. */
        for(;;)
        {
            if(ir(1,950,SENSE)==TRUE&&ir(2,0,NONE)==TRUE&&ir(3,950,SENSE)==TRUE)
            {
                LMotor_1(3);
                RMotor_1(3);
            }
            if(ir(1,0,NONE)==TRUE&&ir(2,950,SENSE)==TRUE&&ir(3,950,SENSE)==TRUE)
            {
                LMotor_1(-7);
                RMotor_1(7);
            }
            if(ir(1,950,SENSE)==TRUE&&ir(2,950,SENSE)==TRUE&&ir(3,0,NONE)==TRUE)
            {
                LMotor_1(7);
                RMotor_1(-7);
            }
            if(ir(1,0,NONE)==TRUE&&ir(2,0,NONE)==TRUE&&ir(3,950,SENSE)==TRUE)
            {
                LMotor_1(-7);
                RMotor_1(7);
            }
            if(ir(1,950,SENSE)==TRUE&&ir(2,0,NONE)==TRUE&&ir(3,0,NONE)==TRUE)
            {
                LMotor_1(7);
                RMotor_1(-7);
            }
            if(ir(1,0,NONE)==TRUE&&ir(2,0,NONE)==TRUE&&ir(3,0,NONE)==TRUE)
            {
                LMotor_1(3);
                RMotor_1(3);
                delay(0,0,0,250);
                break;
            }
        }

        /* Stage 2: same proven steering; stop at the next all-black marker. */
        for(;;)
        {
            if(ir(1,950,SENSE)==TRUE&&ir(2,0,NONE)==TRUE&&ir(3,950,SENSE)==TRUE)
            {
                LMotor_1(3);
                RMotor_1(3);
            }
            if(ir(1,0,NONE)==TRUE&&ir(2,950,SENSE)==TRUE&&ir(3,950,SENSE)==TRUE)
            {
                LMotor_1(-7);
                RMotor_1(7);
            }
            if(ir(1,950,SENSE)==TRUE&&ir(2,950,SENSE)==TRUE&&ir(3,0,NONE)==TRUE)
            {
                LMotor_1(7);
                RMotor_1(-7);
            }
            if(ir(1,0,NONE)==TRUE&&ir(2,0,NONE)==TRUE&&ir(3,950,SENSE)==TRUE)
            {
                LMotor_1(-7);
                RMotor_1(7);
            }
            if(ir(1,950,SENSE)==TRUE&&ir(2,0,NONE)==TRUE&&ir(3,0,NONE)==TRUE)
            {
                LMotor_1(7);
                RMotor_1(-7);
            }
            if(ir(1,0,NONE)==TRUE&&ir(2,0,NONE)==TRUE&&ir(3,0,NONE)==TRUE)
            {
                LMotor_1(0);
                RMotor_1(0);
                break;
            }
        }

        /* Manual RC control, kept as in the supplied working version. */
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
