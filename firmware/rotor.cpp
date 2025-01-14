#include "rotor.h"

#include <math.h>
#include "AccelStepper.h"

void rotor_init(rotor_t *rotor)
{
    tmc2130_init();

    // Stepper motor configuration
    rotor->motor.setMaxSpeed(MAX_SPEED_SPS(rotor->gear_ratio));
    rotor->motor.setAcceleration(MAX_ACCEL_SPSS(rotor->gear_ratio));
    rotor->motor.setMinPulseWidth(20);
}

int rotor_move(rotor_t *rotor, double turns)
{
    if (turns >= MAX_TURNS(rotor->gear_ratio))
        return -1;

    long current_target = rotor->motor.targetPosition();
    current_target += lround(turns * (double)USTEPS_PER_REV * rotor->gear_ratio);
    rotor->motor.moveTo(current_target);

    return 0;
}

void rotor_stop(rotor_t *rotor)
{
    rotor->motor.setCurrentPosition(0);
}

void rotor_enable(rotor_t *rotor, bool enable)
{
    if (!enable)
    {
        rotor_stop(rotor);
    }
    tmc2130_enable(enable);
}

void rotor_try_to_zero_position(rotor_t *rotor)
{
    // See if we are at an opportune time to reset our position
    if (!rotor->motor.isRunning())
    {
        rotor->motor.setCurrentPosition(0);
    }
}
