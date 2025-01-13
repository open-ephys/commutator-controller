#include "motor.h"

#include <math.h>
#include "AccelStepper.h"

void motor_init(AccelStepper *motor)
{
    tmc2130_init();

    // Stepper motor configuration
    motor->setMaxSpeed(MAX_SPEED_SPS);
    motor->setAcceleration(MAX_ACCEL_SPSS);
    motor->setMinPulseWidth(20);
}

void motor_soft_stop(AccelStepper *motor)
{
    motor->setCurrentPosition(0);
}

void motor_hard_stop(AccelStepper *motor)
{
    motor->setAcceleration(1e6);
    motor->stop();
    motor_soft_stop(motor);
    motor->setAcceleration(MAX_ACCEL_SPSS);
}

void motor_enable(AccelStepper *motor, bool enable) 
{ 
    if (!enable) 
    {
        motor_soft_stop(motor);
    }
    tmc2130_enable(enable);
}
