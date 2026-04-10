#pragma once

#include <climits>
#include "AccelStepper.h"
#include "tmc2130.h"

// Turn speed and acceleration (defined at the motor itself)
#define SPEED_RPM 200
#define ACCEL_RPMM 200

// Stepper and driver parameters
#define DETENTS 200
#define USTEPS_PER_REV (DETENTS * USTEPS_PER_STEP)
//#define MAX_TURNS (INT_MAX / USTEPS_PER_REV)

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
void rotor_set_fast_accel(rotor_t *rotor);
void rotor_set_nomimal_accel(rotor_t *rotor);
inline void rotor_stop(rotor_t *rotor) { rotor->motor.stop(); }
inline bool rotor_run(rotor_t *rotor) { return rotor->motor.run(); }