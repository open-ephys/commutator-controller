#include <stdio.h>
#include <iostream>
#include <cmath>
#include <tusb.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"

#include <ArduinoJson.h>

#include "defs.h"
#include "pindefs.h"
#include "ltc4425.h"
#include "is32fl3193.h"
#include "motor.h"
#include "save.h"  // TODO: Does not work properly. Use debug to figure out whats happening

// Versions
#define FIRMWARE_VER        "0.1.0"
#define BOARD_REV           "D"

// 3. Select a commutator type by uncommenting one of the following
//#define COMMUTATOR_TYPE     "SPI"
//const double GEAR_RATIO     = 1.77777777778

#define COMMUTATOR_TYPE     "Single Channel Coax"
const double GEAR_RATIO     = 2.0;

//#define COMMUTATOR_TYPE     "Dual Channel Coax"
//const double GEAR_RATIO     = 3.06666666667

// Holds the current state
Context ctx;
MotorContext mot_ctx {.motor = AccelStepper(AccelStepper::DRIVER, TMC2130_STEP, TMC2130_DIR), .target_turns = 0.0};

void core1_entry() 
{
    while (true)
    {
        mot_ctx.motor.run();
    }
}

int main() 
{
    // Initialize chips
    stdio_init_all();
    motor_enable(false);

    // Load saved state
    //load(ctx);

    rgb_init(ctx);
    ltc4425_init();
    
    // Wait for chips to wake up
    sleep_ms(10);

    // Wait for supercaps to charge
    rgb_set_red();
    rgb_set_breathing(true);
    while(!ltc4425_power_good())
    {
        sleep_ms(10);
    }
    rgb_set_breathing(false);
    rgb_set_auto(ctx);

    // Initialize motor
    motor_init(ctx, mot_ctx);

    // Start the second core with motor.run() as its only task
    multicore_launch_core1(core1_entry);

    // Decode commands and buttons
    while (true)
    {
        if (tud_cdc_available())
        {
            JsonDocument receive;
            auto e = deserializeJson(receive, std::cin);

            if (e) {
                continue;
            }

            auto enable = receive["enable"];
            auto turns = receive["turn"];
            auto led = receive["led"];
            auto print = receive["print"];

            // Enable command
            if (enable.is<bool>()) 
            {
                ctx.commutator_en = enable;

                if (!ctx.commutator_en)
                {
                    motor_hard_stop(mot_ctx);
                } else {
                    motor_enable(true);
                }

                rgb_set_auto(ctx);
                //save(ctx);
            }

            // Turn command
            if (turns.is<double>()) 
            {
                double t = turns.as<double>();
                if (!ctx.commutator_en)
                {
                    JsonDocument doc;
                    doc["error"] = "Cannot move when commutator is disabled";
                    serializeJson(doc, std::cout);
                    std::cout << std::endl;
                }
                else if (std::isnan(t))
                {
                    JsonDocument doc;
                    doc["error"] = "Turn command was NaN";
                    serializeJson(doc, std::cout);
                    std::cout << std::endl;
                }
                else if (std::isinf(t))
                {
                    JsonDocument doc;
                    doc["error"] = "Turn command was Inf";
                    serializeJson(doc, std::cout);
                    std::cout << std::endl;
                }
                else 
                {
                    motor_turn(mot_ctx, t);
                }
            }

            // LED command
            if (led.is<bool>()) 
            {
                ctx.led_on = led;
                rgb_set_auto(ctx);
                //save(ctx);
            }

            // Print command
            if (receive["print"].is<JsonVariant>()) 
            {
                JsonDocument doc;
                doc["type"] = COMMUTATOR_TYPE;
                doc["board_rev"] = BOARD_REV;
                doc["firmware"] = FIRMWARE_VER;
                doc["enable"] = ctx.commutator_en;
                doc["led"] = ctx.led_on;
                doc["steps_to_go"] = mot_ctx.motor.distanceToGo();
                doc["target_steps"] = mot_ctx.motor.targetPosition();
                doc["target_turns"] = mot_ctx.target_turns;
                doc["max_turns"] = MAX_TURNS;
                doc["motor_running"] = mot_ctx.motor.distanceToGo() != 0;
                doc["charge_current"] = ltc4425_charge_current();
                doc["power_good"] = ltc4425_power_good();
                serializeJson(doc, std::cout);
                std::cout << std::endl;
            }
        }
    }
    return 0;
}
