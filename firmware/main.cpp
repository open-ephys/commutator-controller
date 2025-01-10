#include <ArduinoJson.h>
#include <climits>
#include <cmath>
#include <ios>
#include <iostream>
#include <stdio.h>
#include <tusb.h>
#include "hardware/gpio.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include "cap1296.h"
#include "defs.h"
#include "is32fl3193.h"
#include "ltc4425.h"
#include "motor.h"
#include "pindefs.h"
#include "hardware/structs/systick.h"

// Versions
#define FIRMWARE_VER "0.1.0"
#define BOARD_REV "F"

// 3. Select a commutator type by uncommenting one of the following
// #define COMMUTATOR_TYPE     "SPI"
// const double GEAR_RATIO     = 1.77777777778

#define COMMUTATOR_TYPE "Single Channel Coax"
const double GEAR_RATIO = 2.0;

// #define COMMUTATOR_TYPE     "Dual Channel Coax"
// const double GEAR_RATIO     = 3.06666666667

// #define DEBUG

// constants
#define MAX_SERIAL_BUFFER_LENGTH 1024u
const bool unused_placeholder = true;

volatile bool alert_flag; // ISR flag
// thread-safe queues shared between cores
queue_t motor_turn_queue;
queue_t motor_enable_queue;
queue_t motor_stop_queue;

static double turn_motor_if_enabled(double turns, double target_turns, bool enable);
static void core1_entry();
static void reset_serial_buffer(uint8_t *serial_buffer, uint16_t *serial_buffer_index, bool *serial_buffer_has_open_curly);
static void io_alert_irq_callback(unsigned int gpio, long unsigned int events) { alert_flag = true; }

int main()
{
    // Initialize chips
    Context ctx{.enable = false, .led = true};
    double target_turns = 0;
    uint16_t serial_buffer_index = 0;
#ifdef DEBUG
    uint16_t button_counter = 0;
#endif
    uint8_t previous_sensor_input_status, current_sensor_input_status, serial_buffer[MAX_SERIAL_BUFFER_LENGTH], temp_buf;
    memset(serial_buffer, 0, sizeof(serial_buffer));
    bool serial_buffer_has_open_curly = false;
    stdio_init_all();
    rgb_init();
    ltc4425_init();
    queue_init(&motor_turn_queue, sizeof(long), 10);
    queue_init(&motor_enable_queue, sizeof(bool), 10);
    queue_init(&motor_stop_queue, sizeof(bool), 10);
    cap1296_init(); // Capacitative touch sensor
    while (!ltc4425_power_good()){} // Wait for supercaps to charge
    rgb_set_breathing(false);
    rgb_set_auto(ctx);
    multicore_launch_core1(core1_entry);
    gpio_set_irq_enabled_with_callback(CAP1296_ALERT, GPIO_IRQ_EDGE_FALL, true, &io_alert_irq_callback);
    cap1296_clear_int_bit_in_main_control_register();
    // Decode commands and buttons
    while (true)
    {

        // if data is available in the USB serial buffer, process it

        while (tud_cdc_available())
        {
            // Inserting the serial stream directly into deserializeJson() was causing the code that checked for 
            // "{" to hang. This is because that code that was dependent on terminating with eof() byte which 
            // is never received in console stream like this (it can be, but I don't expect users to send it). We manage
            // our own buffer instead. This allows us to know exactly when the message ends to avoid hanging.
            // We check for "{" to begin with because sending a message like "asd{enable:true}" in which a valid JSON is 
            // preceded with gibberish would not count as a valid JSON. 
            if (serial_buffer_index < MAX_SERIAL_BUFFER_LENGTH - 1)
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
                target_turns = 0;
                rgb_set_auto(ctx);
            }

            // LED command
            if (receive["led"].is<bool>())
            {
                ctx.led = receive["led"];
                rgb_set_auto(ctx);
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
                    target_turns = turn_motor_if_enabled(turns, target_turns, ctx.enable);
                }
            }

            // Print command
            if (receive["print"].is<JsonVariant>())
            {
                JsonDocument doc;
                doc["type"] = COMMUTATOR_TYPE;
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

        // if the cap1296 touch sensor detects a button press or release, process it

        if (alert_flag)
        {
            cap1296_clear_int_bit_in_main_control_register();
            current_sensor_input_status = cap1296_read_sensor_input_status_register();
            switch (current_sensor_input_status)
            {
            case LED_BUTTON_PRESS:
                ctx.led = !ctx.led;
                rgb_set_auto(ctx);
                break;
            case ENABLE_BUTTON_PRESS:
                ctx.enable = !ctx.enable;
                queue_add_blocking(&motor_enable_queue, &ctx.enable);
                rgb_set_auto(ctx);
                target_turns = 0;
                break;
            case CW_BUTTON_PRESS:
                target_turns = turn_motor_if_enabled(1000, target_turns, ctx.enable);
                break;
            case CCW_BUTTON_PRESS:
                target_turns = turn_motor_if_enabled(-1000, target_turns, ctx.enable);
                break;
            case BUTTON_RELEASE:
                if (previous_sensor_input_status & (CW_BUTTON_PRESS | CCW_BUTTON_PRESS))
                {
                    queue_add_blocking(&motor_stop_queue, &unused_placeholder);
                    target_turns = 0;
                }
                break;
            }
            previous_sensor_input_status = current_sensor_input_status;
            alert_flag = false;
#ifdef DEBUG
            printf("{button: %02hhx, counter: %i}\n", sensor_input_status, button_counter++);
#endif
        }
    }
    return 0;
}

double turn_motor_if_enabled(double turns, double target_turns, bool enable)
{
    if (enable)
    {
        // Invalid request
        if (abs(turns) > MAX_TURNS)
            return target_turns; // Failure, cant turn this far

        // Relative move
        target_turns += turns;

        if (abs(target_turns) < MAX_TURNS)
        {
            long target_steps = lround(target_turns * (double)USTEPS_PER_REV * GEAR_RATIO);
            queue_add_blocking(&motor_turn_queue, &target_steps);
        } 
        else 
        {
            // Deal with very unlikely case of overflow
            queue_add_blocking(&motor_stop_queue, &unused_placeholder);
            target_turns = turn_motor_if_enabled(target_turns, target_turns, turns); // Restart this routine now that position has been zeroed
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

static void reset_serial_buffer(uint8_t *serial_buffer, uint16_t *serial_buffer_index, bool *serial_buffer_has_open_curly)
{
    memset(serial_buffer, 0, *serial_buffer_index);
    *serial_buffer_index = 0;
    *serial_buffer_has_open_curly = false;
}

static void core1_entry()
{
    MotorContext mot_ctx{.motor = AccelStepper(AccelStepper::DRIVER, TMC2130_STEP, TMC2130_DIR), .target_turns = 0.0};
    motor_enable(mot_ctx, false);
    motor_init(mot_ctx);
    long queue_long_buffer;
    bool queue_bool_buffer;
    while (true)
    {
        if (queue_try_remove(&motor_turn_queue, &queue_long_buffer))
        {
            mot_ctx.motor.moveTo(queue_long_buffer);
        }
        else if (queue_try_remove(&motor_stop_queue, &queue_bool_buffer))
        {
            motor_soft_stop(mot_ctx);
        }
        else if (queue_try_remove(&motor_enable_queue, &queue_bool_buffer))
        {
            motor_enable(mot_ctx, (bool)queue_bool_buffer);
        }
        mot_ctx.motor.run();
    }
}