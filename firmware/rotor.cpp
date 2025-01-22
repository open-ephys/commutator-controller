#include "rotor.h"

#include <math.h>
#include "AccelStepper.h"

void rotor_init(rotor_t *rotor)
{
    tmc2130_init();

    // Stepper motor configuration
    rotor->motor.setMaxSpeed(MAX_SPEED_SPS(rotor->gear_ratio));
    rotor->motor.setAcceleration(MAX_ACCEL_SPSS(rotor->gear_ratio));
    rotor->motor.setMinPulseWidth(2);
}

int rotor_move(rotor_t *rotor, double turns)
{
    if (turns >= MAX_TURNS(rotor->gear_ratio))
        return -1;

    rotor->target_position += turns;
    long target_position_steps = lround(rotor->target_position * (double)USTEPS_PER_REV * rotor->gear_ratio);
    rotor->motor.moveTo(target_position_steps);

    return 0;
}