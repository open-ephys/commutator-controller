#include "rotor.h"

#include <math.h>
#include "AccelStepper.h"

void rotor_enable(rotor_t *rotor, bool enable)
{
    if (!enable)
    {
        rotor_stop_and_reset(rotor);
    }

    tmc2130_enable(enable);
}

void rotor_init(rotor_t *rotor)
{
    tmc2130_init();

    // Stepper motor configuration
    rotor->motor.setMaxSpeed(MAX_SPEED_SPS);
    rotor->motor.setAcceleration(MAX_ACCEL_SPSS);
    rotor->motor.setMinPulseWidth(2);
}

int rotor_move(rotor_t *rotor, double turns)
{
    if (std::isinf(turns))
    {
        double dir = std::signbit(turns) ? -1.0 : 1.0;
        rotor->target_position = (rotor->motor.currentPosition() / (double)USTEPS_PER_REV / rotor->gear_ratio) + (dir * 100.0);
    }

    else if (fabs(turns) >= MAX_TURNS)
        return -1;

    else 
        rotor->target_position += turns;
    
    long target_position_steps = lround(rotor->target_position * (double)USTEPS_PER_REV * rotor->gear_ratio);
    rotor->motor.moveTo(target_position_steps);

    return 0;
}