#include "motor.h"

#include <math.h>

void motor_init(Context &ctx, MotorContext &mot_ctx)
{
    tmc2130_init();

    // Stepper motor configuration
    mot_ctx.target_turns = 0;
    mot_ctx.motor.setMaxSpeed(MAX_SPEED_SPS);
    mot_ctx.motor.setAcceleration(MAX_ACCEL_SPSS);
    mot_ctx.motor.setMinPulseWidth(20);
    motor_enable(ctx.enable);
}

void motor_soft_stop(MotorContext &mot_ctx)
{
    mot_ctx.motor.setCurrentPosition(0);
    mot_ctx.target_turns = 0.0;
}

void motor_hard_stop(MotorContext &mot_ctx)
{
    mot_ctx.motor.setAcceleration(1e6);
    mot_ctx.motor.stop();
    mot_ctx.motor.runToPosition();
    mot_ctx.motor.setCurrentPosition(0);
    mot_ctx.target_turns = 0.0;
    mot_ctx.motor.setAcceleration(MAX_ACCEL_SPSS);
    motor_enable(false);
}

void motor_turn(MotorContext &mot_ctx, double turns)
{
    // Invalid request
    if (abs(turns) > MAX_TURNS)
        return; // Failure, cant turn this far

    // Relative move
    mot_ctx.target_turns += turns;

    if (abs(mot_ctx.target_turns) < MAX_TURNS)
    {
        mot_ctx.motor.moveTo(lround(mot_ctx.target_turns * (double)USTEPS_PER_REV * GEAR_RATIO));
    } else {
        // Deal with very unlikely case of overflow
        motor_soft_stop(mot_ctx);
        motor_turn(mot_ctx, turns); // Restart this routine now that position has been zeroed
    }
}