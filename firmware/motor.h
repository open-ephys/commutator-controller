#pragma once

#include <climits>
#include "defs.h"
#include "tmc2130.h"

struct Context;

extern const double GEAR_RATIO;

// Stepper parameters
static const int DETENTS = 200;
static const int USTEPS_PER_STEP = 8;
static const int USTEPS_PER_REV = (DETENTS * USTEPS_PER_STEP);
static const int MAX_TURNS = (INT_MAX / USTEPS_PER_REV / GEAR_RATIO);

// Turn speed and acceleration
static const int SPEED_RPM = 100;
static const int ACCEL_RPMM = 150;
static const double MAX_SPEED_SPS = USTEPS_PER_REV * GEAR_RATIO * SPEED_RPM / 60.0;
static const double  MAX_ACCEL_SPSS = USTEPS_PER_REV * GEAR_RATIO * ACCEL_RPMM / 60.0;

void motor_enable(MotorContext &mot_ctx, bool enable);
void motor_init(MotorContext &mot_ctx);
void motor_soft_stop(MotorContext &mot_ctx);
void motor_hard_stop(MotorContext &mot_ctx);
void motor_turn(MotorContext &mot_ctx, double turns);
