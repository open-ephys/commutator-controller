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
#include "cap1296.h"
#include "defs.h"
#include "is32fl3193.h"
#include "ltc4425.h"
#include "motor.h"
#include "pindefs.h"

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

#define MAX_MESSAGE_LENGTH 1024

volatile bool alert_flag;
uint8_t message[MAX_MESSAGE_LENGTH];
uint16_t message_index = 0;
Context ctx{.enable = false, .led = true};
MotorContext mot_ctx{.motor = AccelStepper(AccelStepper::DRIVER, TMC2130_STEP, TMC2130_DIR), .target_turns = 0.0};
#ifdef DEBUG
uint16_t button_counter = 0;
#endif

static void process_serial_input(uint8_t sensor_input_status);
static uint8_t process_button_input(uint8_t sensor_input_status);
static void turn_motor_if_enabled(double turns);
static bool check_for_curly_bracket();
static void core1_entry();
static void io_alert_irq_callback(unsigned int gpio, long unsigned int events) { alert_flag = true; }

// bool repeating_timer_callback(__unused struct repeating_timer *t) {
//     if (mot_ctx.motor.isRunning()){
//         mot_ctx.motor.run();
//     }
// }

int main()
{
    // Initialize chips
    uint8_t sensor_input_status;
    stdio_init_all();
    motor_enable(mot_ctx, ctx.enable);
    rgb_init();
    ltc4425_init();
    cap1296_init(); // Capacitative touch sensor
    // Wait for supercaps to charge
    while (!ltc4425_power_good())
    {
        sleep_ms(10);
    }

    rgb_set_breathing(false);
    rgb_set_auto(ctx);
    motor_init(ctx, mot_ctx); // Initialize motor
    // Start the second core with motor.run() as its only task
    multicore_launch_core1(core1_entry);
    cap1296_clear_int_bit_in_main_control_register();
    // struct repeating_timer timer;
    // add_repeating_timer_us(-20, repeating_timer_callback, NULL, &timer);
    // Decode commands and buttons
    while (true)
    {
        mot_ctx.motor.run();
    }
    return 0;
}

void process_serial_input(uint8_t sensor_input_status)
{
    if (message_index < MAX_MESSAGE_LENGTH - 1)
    {
        uint8_t temp_buf = getchar();
        bool message_already_has_open_curly = false;
        for (uint16_t i = 0; i < message_index ; i++)
        {
            if (message[i] == '{')
            {
                message_already_has_open_curly = true;
            }
        }
        if (temp_buf == '{' | message_already_has_open_curly)
        {
            message[message_index++] = temp_buf;
        }
        else
        {
            return;
        }
        
    }
    else
    {
        printf("{error: input buffer overflow}\n");
        memset(message, 0, MAX_MESSAGE_LENGTH);
        message_index = 0;
    }

    JsonDocument receive;
    auto error = deserializeJson(receive, message, message_index);

    if (error.code())
    {
        return;
    }
    memset(message, 0, message_index);
    message_index = 0;

    // Enable command
    if (receive["enable"].is<bool>())
    {
        ctx.enable = receive["enable"];
        motor_enable(mot_ctx, ctx.enable);
        rgb_set_auto(ctx);
#ifdef DEBUG
        printf("{enable: %d}\n", ctx.enable);
#endif
    }

    // LED command
    if (receive["led"].is<bool>())
    {
        ctx.led = receive["led"];
        rgb_set_auto(ctx);
#ifdef DEBUG
        printf("{led: %d}\n", ctx.led);
#endif
    }

    // Turn command, but don't let this command override current button presses
    if (receive["turn"].is<double>() & !(sensor_input_status & (CW_BUTTON_PRESS | CCW_BUTTON_PRESS)))
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
            turn_motor_if_enabled(turns);
        }
#ifdef DEBUG
        printf("{turn: %g}\n", turns);
#endif
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

uint8_t process_button_input(uint8_t previous_sensor_input_status)
{
    cap1296_clear_int_bit_in_main_control_register();
    uint8_t sensor_input_status = cap1296_read_sensor_input_status_register();

#ifdef DEBUG
    if (sensor_input_status)
    {
        button_counter++;
    }
    printf("{button: %02hhx, counter: %i}\n", sensor_input_status, button_counter);
#endif

    switch (sensor_input_status)
    {
    case LED_BUTTON_PRESS:
        ctx.led = !ctx.led;
        rgb_set_auto(ctx);
        break;
    case ENABLE_BUTTON_PRESS:
        ctx.enable = !ctx.enable;
        motor_enable(mot_ctx, ctx.enable);
        rgb_set_auto(ctx);
        break;
    case CW_BUTTON_PRESS:
        turn_motor_if_enabled(1000);
        break;
    case CCW_BUTTON_PRESS:
        turn_motor_if_enabled(-1000);
        break;
    case BUTTON_RELEASE:
        if (previous_sensor_input_status & (CW_BUTTON_PRESS | CCW_BUTTON_PRESS))
        {
            motor_soft_stop(mot_ctx);
        }
        break;
    }
    return sensor_input_status;
}

void turn_motor_if_enabled(double turns)
{
    if (ctx.enable)
    {
        if (!mot_ctx.motor.isRunning())
        {
            motor_soft_stop(mot_ctx); // avoid overflow condition
        }
        motor_turn(mot_ctx, turns);
    }
    else
    {
        JsonDocument doc;
        doc["error"] = "Turn command received while disabled";
        serializeJson(doc, std::cout);
        std::cout << std::endl;
    }
}

static void core1_entry()
{
    gpio_set_irq_enabled_with_callback(CAP1296_ALERT, GPIO_IRQ_EDGE_FALL, true, &io_alert_irq_callback);
    uint8_t sensor_input_status;
    while (true)
    {
        if (tud_cdc_available())
        {
            process_serial_input(sensor_input_status);
        }
        if (alert_flag)
        {
            sensor_input_status = process_button_input(sensor_input_status);
            alert_flag = false;
        }
        
    }
}