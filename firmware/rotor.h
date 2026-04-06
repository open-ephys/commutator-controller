#pragma once

#include <climits>
#include "AccelStepper.h"
#include "tmc2130.h"

// Stepper and driver parameters
#define DETENTS 200
#define USTEPS_PER_STEP 64
#define USTEPS_PER_REV (DETENTS * USTEPS_PER_STEP)
#define MAX_TURNS (INT_MAX / USTEPS_PER_REV)

// Turn speed and acceleration
#define SPEED_RPM 200
#define ACCEL_RPMM 100
#define MAX_SPEED_SPS (USTEPS_PER_REV * SPEED_RPM / 60.0L)
#define MAX_ACCEL_SPSS (USTEPS_PER_REV * ACCEL_RPMM / 60.0L)
#define MAX_ACCEL_SPSS_BUTTON (2 * MAX_ACCEL_SPSS)

typedef struct rotor_t
{
    AccelStepper motor;
    double gear_ratio;
    double target_position;
} rotor_t;

static inline void rotor_stop_and_reset(rotor_t *rotor)
{
    rotor->target_position = 0.0;
    rotor->motor.setCurrentPosition(0);
}

void rotor_enable(rotor_t *rotor, bool enable);
void rotor_init(rotor_t *rotor);
int rotor_move(rotor_t *rotor, double turns);