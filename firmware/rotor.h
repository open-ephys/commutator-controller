#pragma once

#include <climits>
#include "AccelStepper.h"
#include "tmc2130.h"

// Stepper and driver parameters
#define DETENTS 200
#define USTEPS_PER_STEP 8
#define USTEPS_PER_REV (DETENTS * USTEPS_PER_STEP)
#define MAX_TURNS(gear_ratio) (INT_MAX / USTEPS_PER_REV / gear_ratio)

// Turn speed and acceleration
#define SPEED_RPM 100
#define ACCEL_RPMM 150
#define MAX_SPEED_SPS(gear_ratio) (USTEPS_PER_REV * gear_ratio * SPEED_RPM / 60.0L)
#define MAX_ACCEL_SPSS(gear_ratio) (USTEPS_PER_REV * gear_ratio * ACCEL_RPMM / 60.0L)

typedef struct rotor_t {
    AccelStepper motor;
    double gear_ratio;
} rotor_t;

void rotor_init(rotor_t *rotor);
void rotor_enable(rotor_t *rotor, bool enable);
int rotor_move(rotor_t *rotor, double turns);
void rotor_stop(rotor_t *rotor);
void rotor_try_to_zero_position(rotor_t *rotor);
