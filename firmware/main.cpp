#include <ArduinoJson.h>
#include <climits>
#include <cmath>
#include <ios>
#include <iostream>
#include <stdio.h>
#include <tusb.h>
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include "cap1296.h"
#include "defs.h"
#include "io.h"
#include "is32fl3193.h"
#include "ltc4425.h"
#include "motor.h"
#include "pindefs.h"

//////// DEBUG ////////

// #define DEBUG

////////// VERSIONING //////////

#define FIRMWARE_VER "0.1.0"
#define BOARD_REV "G" 

////////// CONSTANTS //////////

#define MAX_SERIAL_BUFFER_LENGTH 1024u
// isr flag for capacitative touch sensing
volatile bool alert_flag = false;
// shared state between cores: thread-safe queues
queue_t motor_turn_queue;
queue_t motor_enable_queue;
queue_t motor_stop_queue;
// send this value to core1 through motor_stop_queue whenever a stop command is received 
const bool stop_flag = true;
// configure gear ratio through `picotool config`
bi_decl(bi_ptr_string(0, 0, gear_ratio, "2.0", 32));
double GEAR_RATIO = 2.0;

///////// FUNCTION DECLARATIONS /////////

static double turn_command(double turns, double target_turns, bool enable);
static void turn_button(bool direction, bool enable);
static void core1_entry();
static void reset_serial_buffer(uint8_t *serial_buffer, uint16_t *serial_buffer_index, bool *serial_buffer_has_open_curly);

///////// MAIN /////////

int main()
{
    // Initialize variables
#ifdef DEBUG
    uint16_t button_counter = 0;
#endif
    struct Context {
        bool enable = false;
        bool led = true;
        double target_turns = 0.0;
    } ctx;
    uint16_t serial_buffer_index = 0;
    uint8_t previous_sensor_input_status, current_sensor_input_status, temp_buf = 0;
    uint8_t serial_buffer[MAX_SERIAL_BUFFER_LENGTH] = {0};
    bool serial_buffer_has_open_curly = false;
    GEAR_RATIO = atof(gear_ratio);
    queue_init(&motor_turn_queue, sizeof(long), 10);
    queue_init(&motor_enable_queue, sizeof(bool), 10);
    queue_init(&motor_stop_queue, sizeof(bool), 10);
    
    // Initialize chips
    io_init();
    rgb_init();
    rgb_set_auto(ctx.enable, ctx.led);
    rgb_set_breathing(true);
    ltc4425_init();
    cap1296_init();
    while (!ltc4425_power_good()){} // Wait for supercaps to charge
    rgb_set_breathing(false);
    rgb_set_auto(ctx.enable, ctx.led);

    // Launch asynchronous processes 
    multicore_launch_core1(core1_entry); // Launch core1
    gpio_set_irq_enabled_with_callback(CAP1296_ALERT, GPIO_IRQ_EDGE_FALL, true, &io_alert_irq_callback); // Enable isr
    cap1296_clear_int_bit_in_main_control_register(); // Clear any button touches registered during initialization
    alert_flag = false;

    // Decode commands and buttons
    while (true)
    {

        // if data is available in the USB serial buffer, process it

        while (tud_cdc_available())
        {
            // Inserting the serial stream directly into deserializeJson() was causing the code that checked for 
            // "{" to hang. This is because that code that was dependent on terminating with eof() byte which 
            // is never received in a console stream like this (it can be, but we don't expect users to send it). We manage
            // our own buffer instead. This allows us to know exactly when the message ends to avoid hanging.
            // We check for "{" to begin with because sending a message like "asd{enable:true}" in which a valid JSON is 
            // preceded with gibberish would not count as a valid JSON. 
            if (serial_buffer_index < MAX_SERIAL_BUFFER_LENGTH)
            {
                temp_buf = getchar();
                if (serial_buffer_has_open_curly)
                {
                    serial_buffer[serial_buffer_index++] = temp_buf;
                }
                else if (temp_buf == '{')
                {
                    serial_buffer_has_open_curly = true;
                    serial_buffer[serial_buffer_index++] = temp_buf;
                }
                else
                {
                    break;
                }
                
            }
            else
            {
                printf("{error: input buffer overflow}\n");
                reset_serial_buffer(serial_buffer, &serial_buffer_index, &serial_buffer_has_open_curly);
            }

            JsonDocument receive;
            auto error = deserializeJson(receive, serial_buffer, serial_buffer_index, DeserializationOption::NestingLimit(2));
            
            // If the `temp_buf == '-'` is omitted, negative turns are omitted.
            // Before the code just checked for any non-zero `error.code()` and hit `break;`. However, this 
            // bricks serial communication if an invalid JSON is received. Every subsequent DeserializeJson would
            // find this invalid json and proceed no further in the serial_buffer to find a subsequent potentially 
            // valid JSON received later on, I think
            if (error.code() == DeserializationError::IncompleteInput || temp_buf == '-')
            {
                break;
            }
            else if (error.code())
            {
                reset_serial_buffer(serial_buffer, &serial_buffer_index, &serial_buffer_has_open_curly);
                break;
            }

            reset_serial_buffer(serial_buffer, &serial_buffer_index, &serial_buffer_has_open_curly);

            // Enable command
            if (receive["enable"].is<bool>())
            {
                ctx.enable = receive["enable"];
                queue_add_blocking(&motor_enable_queue, &ctx.enable);
                rgb_set_auto(ctx.enable, ctx.led);
            }

            // LED command
            if (receive["led"].is<bool>())
            {
                ctx.led = receive["led"];
                rgb_set_auto(ctx.enable, ctx.led);
            }

            // Turn command, but don't let this command override current button presses
            if (receive["turn"].is<double>() & !(current_sensor_input_status & (CW_BUTTON_PRESS | CCW_BUTTON_PRESS)))
            {
                double turns = receive["turn"];
                if (std::isnan(turns))
                {
                    JsonDocument doc;
                    doc["error"] = "NaN turn command";
                    serializeJson(doc, std::cout);
                    std::cout << std::endl;
                }
                else if (std::isinf(turns))
                {
                    JsonDocument doc;
                    doc["error"] = "Inf turn command";
                    serializeJson(doc, std::cout);
                    std::cout << std::endl;
                }
                else
                {
                    ctx.target_turns = turn_command(turns, ctx.target_turns, ctx.enable);
                }
            }

            // Print command
            if (receive["print"].is<JsonVariant>())
            {
                JsonDocument doc;
                doc["gear_ratio"] = GEAR_RATIO;
                doc["board_rev"] = BOARD_REV;
                doc["firmware"] = FIRMWARE_VER;
                doc["enable"] = ctx.enable;
                doc["led"] = ctx.led;
                doc["charge_current"] = ltc4425_charge_current();
                doc["power_good"] = ltc4425_power_good();
                serializeJson(doc, std::cout);
                std::cout << std::endl;
            }
#ifdef DEBUG
            serializeJson(receive, std::cout);
            std::cout << std::endl;
#endif
        }

        // If the cap1296 touch sensor detects a button press or release, process it

        if (alert_flag)
        {
            cap1296_clear_int_bit_in_main_control_register();
            current_sensor_input_status = cap1296_read_sensor_input_status_register();
            switch (current_sensor_input_status)
            {
            case LED_BUTTON_PRESS:
                ctx.led = !ctx.led;
                rgb_set_auto(ctx.enable, ctx.led);
                break;
            case ENABLE_BUTTON_PRESS:
                ctx.enable = !ctx.enable;
                queue_add_blocking(&motor_enable_queue, &ctx.enable);
                rgb_set_auto(ctx.enable, ctx.led);
                ctx.target_turns = 0;
                break;
            case CW_BUTTON_PRESS:
                turn_button(false, ctx.enable);
                break;
            case CCW_BUTTON_PRESS:
                turn_button(true, ctx.enable);
                break;
            case BUTTON_RELEASE:
                if (previous_sensor_input_status & (CW_BUTTON_PRESS | CCW_BUTTON_PRESS))
                {
                    queue_add_blocking(&motor_stop_queue, &stop_flag);
                    ctx.target_turns = 0; // both target_turns and accel stepper's absolute position-tracking variable
                }                         // (_currentPosition) reset to zero after manually adjusting commutator with buttons
                break;
            }
            previous_sensor_input_status = current_sensor_input_status;
            alert_flag = false;
#ifdef DEBUG
            printf("{button: %02hhx, counter: %i}\n", current_sensor_input_status, button_counter++);
#endif
        }
    }
    return 0;
}

double turn_command(double turns, double target_turns, bool enable)
{
    if (enable)
    {
        // Invalid request
        if (abs(turns) > MAX_TURNS)
            return target_turns; // Failure, cant turn this far

        target_turns += turns; // Relative move

        if (abs(target_turns) < MAX_TURNS)
        {
            long target_steps = lround(target_turns * (double)USTEPS_PER_REV * GEAR_RATIO);
            queue_add_blocking(&motor_turn_queue, &target_steps);
        } 
        else 
        {
            queue_add_blocking(&motor_stop_queue, &stop_flag); // Deal with very unlikely case of overflow
            target_turns = turn_command(target_turns, target_turns, turns); // Restart this routine now that position has been zeroed
        }
    }
    else
    {
        JsonDocument doc;
        doc["error"] = "Turn command received while disabled";
        serializeJson(doc, std::cout);
        std::cout << std::endl;
    }
    return target_turns;
}

void turn_button(bool direction, bool enable)
{
    if (enable)
    {
        long target_steps;
        if (direction)
        {
            target_steps = lround(100 * (double)USTEPS_PER_REV * GEAR_RATIO);
        } 
        else 
        {
            target_steps = lround(-100 * (double)USTEPS_PER_REV * GEAR_RATIO);
        }
        queue_add_blocking(&motor_turn_queue, &target_steps);
    }
    else
    {
        JsonDocument doc;
        doc["error"] = "Turn command received while disabled";
        serializeJson(doc, std::cout);
        std::cout << std::endl;
    }
    return;
}

static void reset_serial_buffer(uint8_t *serial_buffer, uint16_t *serial_buffer_index, bool *serial_buffer_has_open_curly)
{
    memset(serial_buffer, 0, MAX_SERIAL_BUFFER_LENGTH);
    *serial_buffer_index = 0;
    *serial_buffer_has_open_curly = false;
}

static void core1_entry()
{
    AccelStepper motor = AccelStepper(AccelStepper::DRIVER, TMC2130_STEP, TMC2130_DIR);
    long queue_long_buffer;
    bool queue_bool_buffer;
    motor_init(&motor);
    while (true)
    {
        if (queue_try_remove(&motor_turn_queue, &queue_long_buffer))
        {
            motor.moveTo(queue_long_buffer);
        }
        else if (queue_try_remove(&motor_stop_queue, &queue_bool_buffer))
        {
            // This method is not a standard part of the AccelStepper library.
            // Both target_turns and accel stepper's absolute position-tracking variable (_currentPosition) 
            motor.stopAndResetPosition(); // reset to zero after manually adjusting commutator with buttons
        }                                 
        else if (queue_try_remove(&motor_enable_queue, &queue_bool_buffer))
        {
            motor_enable(&motor, queue_bool_buffer);
        }
        motor.run();
    }
}