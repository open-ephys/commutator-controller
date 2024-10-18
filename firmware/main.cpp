#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"

#include "pindefs.h"
#include "ltc4425.h"
#include "is32fl3193.h"
#include "tmc2130.h"
#include "parser.h"

#include "accelstepper/AccelStepper.h"
#include "tinyjson/tiny-json.h"


// Firmware Version
#define FIRMWARE_VER        "0.1.0"

// 1. Uncomment to continuously dump button press data over Serial
//#define DEBUG

// 3. Select a commutator type by uncommenting one of the following
//#define COMMUTATOR_TYPE     "SPI Rev. A"
//#define GEAR_RATIO          1.77777777778

#define COMMUTATOR_TYPE     "Single Channel Coax Rev. A"
#define GEAR_RATIO          2.0

//#define COMMUTATOR_TYPE     "Dual Channel Coax Rev. A"
//#define GEAR_RATIO          3.06666666667

// Stepper parameters
#define DETENTS             200
#define USTEPS_PER_STEP     8
#define USTEPS_PER_REV      (DETENTS * USTEPS_PER_STEP)
#define MAX_TURNS           (2147483647 / USTEPS_PER_REV / GEAR_RATIO)

// Turn speed and acceleration
#define SPEED_RPM           100
#define ACCEL_RPMM          150
#define MAX_SPEED_SPS       ((float)USTEPS_PER_REV * GEAR_RATIO * SPEED_RPM / 60.0)
#define MAX_ACCEL_SPSS      ((float)USTEPS_PER_REV * GEAR_RATIO * ACCEL_RPMM / 60.0)



AccelStepper motor(AccelStepper::DRIVER, TMC2130_STEP, TMC2130_DIR);
double target_turns = 0;


void soft_stop()
{
    motor.setCurrentPosition(0);
    target_turns = 0.0;
}

void turn_motor(double turns)
{
    // Invalid request
    if (abs(turns) > MAX_TURNS)
        return; // Failure, cant turn this far

    // Relative move
    target_turns += turns;

    if (abs(target_turns) < MAX_TURNS)
    {
        motor.moveTo(lround(target_turns * (double)USTEPS_PER_REV * GEAR_RATIO));
    } else {
        // Deal with very unlikely case of overflow
        soft_stop();
        turn_motor(turns); // Restart this routine now that position has been zeroed
    }
}


void core1_entry() 
{
    while (true)
    {
        motor.run();
    }
}



int main() 
{
    // Initialize chips
    stdio_init_all();
    ltc4425_init();
    rgb_init();
    tmc2130_init();

    // Wait for chips to wake up
    sleep_ms(10);

    // Wait for supercaps to charge
    RGB_SET_RED
    rgb_set_breathing(true);
    while(!gpio_get(LTC4425_nPOW_FAIL))
    {
        printf("Charge current: %f A\n", ltc4425_charge_current());
        sleep_ms(10);
    }
    rgb_set_breathing(false);
    RGB_SET_GREEN

    // Stepper motor configuration
    double target_turns = 0;
    motor.setMaxSpeed(MAX_SPEED_SPS);
    motor.setAcceleration(MAX_ACCEL_SPSS);
    motor.setMinPulseWidth(20);
    tmc2130_enable(1);

    // Start the second core with motor.run() as its only task
    multicore_launch_core1(core1_entry);

    // When using serial monitor
    // -- set ending to CR
    // -- send this {"turn":1.0} and it will turn
    while (true)
    {
        json_t const *root = NULL;
        int rc = read_json_from_stdin(root);
        if (rc)
        {
            printf("Invalid JSON formatting: %d\n", rc);
            continue;
        }

        double turns = 0;
        rc = parse_turns(root, &turns);
        if (!rc && !isnan(turns)) {
            turn_motor(turns);
        } else {
            printf("Invalid JSON formatting: %d\n", rc);
        }
    }

    return 0;
}
