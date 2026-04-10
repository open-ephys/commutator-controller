#include "rotor.h"
#include "tmc2130.h"

#include <math.h>
#include "AccelStepper.h"

#define MAX_SPEED_SPS (USTEPS_PER_REV * SPEED_RPM / 60.0L)
#define MAX_ACCEL_SPSS (USTEPS_PER_REV * ACCEL_RPMM / 60.0L)
#define MAX_ACCEL_SPSS_BUTTON (2 * MAX_ACCEL_SPSS)

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
    rotor->target_position += turns;
    long target_position_steps = lround(rotor->target_position * (double)USTEPS_PER_REV * rotor->gear_ratio);
    rotor->motor.moveTo(target_position_steps);

    return 0;
}

void rotor_set_fast_accel(rotor_t *rotor)
{
    rotor->motor.setAcceleration(MAX_ACCEL_SPSS_BUTTON);
}

void rotor_set_nomimal_accel(rotor_t *rotor)
{
     rotor->motor.setAcceleration(MAX_ACCEL_SPSS);
}