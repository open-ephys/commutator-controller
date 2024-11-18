#include <algorithm>
#include <ArduinoJson.h>
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <tusb.h>
#include "hardware/flash.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "cap1296.h"
#include "defs.h"
#include "io.h"
#include "is32fl3193.h"
#include "ltc4425.h"
#include "motor.h"
#include "pindefs.h"

// #define DEBUG

// Versions
#define FIRMWARE_VER "0.1.0"
#define BOARD_REV "E"

// 3. Select a commutator type by uncommenting one of the following
// #define COMMUTATOR_TYPE     "SPI"
// const double GEAR_RATIO     = 1.77777777778

#define COMMUTATOR_TYPE "Single Channel Coax"
const double GEAR_RATIO = 2.0;

// #define COMMUTATOR_TYPE     "Dual Channel Coax"
// const double GEAR_RATIO     = 3.06666666667

#define MAX_MESSAGE_LENGTH 1024

volatile uint8_t alert_flag;
Context ctx;
MotorContext mot_ctx{.motor = AccelStepper(AccelStepper::DRIVER, TMC2130_STEP, TMC2130_DIR), .target_turns = 0.0};

void print_diagnostics();
void print_context(Context context, MotorContext motor_context);
void turn_cmd(Context context, MotorContext motor_context, double t);
void write_context_to_flash(Context context);
void update_context(Context context, MotorContext motor_context);

// void __no_inline_not_in_flash_func(core1_entry)()
void core1_entry()
{
    multicore_lockout_victim_init();
    while (true)
    {
        mot_ctx.motor.run();
    }
}

uint8_t extract_touch_data(uint8_t previous_button_press, uint8_t touch_status_register)
{
    // This logic is required to deal with the fact that the release from the previously touched button is not cleared
    if (touch_status_register & (touch_status_register - 1))
    { // If more than one bit is set, mask out the bit that corresponds to the previously button
        return (previous_button_press ^ touch_status_register) & touch_status_register;
    }
    else
    { // If one bit is set, then the current button press is the same as the previous button press
        return touch_status_register;
    }
}

int main()
{
    // Declare or define local variables
    uint8_t touch_status_register, previous_button_press, current_button_press; // for button logic
    uint8_t context_flag, turn_flag, write_context_flag;                        // flags
    uint8_t contains_open_bracket, error_code;                                  // for parsing incoming data
    char message[MAX_MESSAGE_LENGTH] = {0};
    uint16_t message_index;
    double t;
    JsonDocument receive_doc;
    // Initialize hardware
    io_init();      // RP2040 io
    rgb_init();     // (IS3FL3193) RGB LED driver
    ltc4425_init(); // Super capacitor charger
    cap1296_init(); // Capacitative touch sensor
    ctx = *(Context *)(XIP_BASE + FLASH_TARGET_OFFSET); // Read context from flash
    motor_init(ctx, mot_ctx);                           // (TMC2130) motor driver
    sleep_ms(10);                                       // Wait for chips to wake up
    rgb_set_breathing(false);
    rgb_update(ctx);
    multicore_launch_core1(core1_entry); // Start the second core with motor.run() as its only task
    cap1296_clear_touch_status();
#ifdef DEBUG
    while (!tud_cdc_connected()){}
    uint8_t button_counter;
    print_diagnostics();
#endif
    // Decode commands and buttons
    while (true)
    {

        // If data is available in the serial port
        if (tud_cdc_available())
        { 
            if (message_index < MAX_MESSAGE_LENGTH - 1)
            {
                message[message_index++] = getchar();
            }
            else
            {
                printf("Invalid, too large");
                message_index = 0;
            }
            auto error = deserializeJson(receive_doc, message);
            contains_open_bracket = 0;
            for (uint16_t i = 0; i < message_index; i++)
            {
                if (message[i] == '{')
                {
                    contains_open_bracket = 1;
                }
            }
            if (contains_open_bracket == 0)
            {
                error_code = 3;
            }
            else
            {
                error_code = error.code();
            }
#ifdef DEBUG
            printf("message: %.*s\n", message_index, message);
            printf("error: %d\n", error_code);
#endif
            if (error_code == DeserializationError::Ok)
            {
#ifdef DEBUG
                printf("Valid JSON: %.*s\n", message_index, message);
#endif
                memset(message, 0, message_index);
                message_index = 0;
                for (JsonPair kv : receive_doc.as<JsonObject>())
                {
                    if (!strcmp(kv.key().c_str(), "turn"))
                    {
                        if (kv.value().is<double>())
                        {
                            t = kv.value();
                            turn_flag = 1;
                            printf("%s: %g\n", kv.key().c_str(), t);
                        }
                        else
                        {
                            printf("Invalid value for \"%s\"\n", kv.key().c_str());
                        }
                        break;
                    }
                    else if (!strcmp(kv.key().c_str(), "led"))
                    {
                        if (kv.value().is<bool>())
                        {
                            ctx.led = kv.value();
                            context_flag = 1;
                            printf("%s: %d\n", kv.key().c_str(), ctx.led);
                        }
                        else
                        {
                            printf("Invalid value for \"%s\"\n", kv.key().c_str());
                        }
                        break;
                    }
                    else if (!strcmp(kv.key().c_str(), "enable"))
                    {
                        if (kv.value().is<bool>())
                        {
                            ctx.enable = kv.value();
                            context_flag = 1;
                            printf("%s: %d\n", kv.key().c_str(), ctx.enable);
                        }
                        else
                        {
                            printf("Invalid value for \"%s\"\n", kv.key().c_str());
                        }
                        break;
                    }
                    else if (!strcmp(kv.key().c_str(), "print"))
                    {
                        print_context(ctx, mot_ctx);
                    }
                    else
                    {
                        printf("Invalid key: \"%s\"\n", kv.key().c_str());
                    }
                }
            }
            else if (error_code == DeserializationError::InvalidInput)
            {
                std::string message_string(message);
                message_string.erase(std::remove_if(message_string.begin(), message_string.end(), isspace) - 1, message_string.end());
                if (!strcmp(message_string.c_str(), "{print:"))
                {
                    print_context(ctx, mot_ctx);
                }
                else
                {
                    printf("Invalid JSON: %.*s\n", message_index, message);
                }
                memset(message, 0, message_index);
                message_index = 0;
            }
        }

        // If alert_flag is set by interrupt callback when a button is touched
        if (alert_flag == 1)
        {   
            alert_flag = 0;
            touch_status_register = cap1296_read_touch_status();
            cap1296_clear_touch_status();
            current_button_press = extract_touch_data(previous_button_press, touch_status_register);
            previous_button_press = current_button_press;
#ifdef DEBUG
            printf("State: %02hhx, Button: %02hhx, Count: %u\n", touch_status_register, current_button_press, ++button_counter);
#endif
            switch (current_button_press)
            {
            case 0x04: // top button
                ctx.led = !ctx.led;
                context_flag = 1;
                break;
            case 0x02: // bottom button
                ctx.enable = !ctx.enable;
                context_flag = 1;
                break;
            case 0x01: // left button
                turn_flag = 1;
                t = -1.0;
                break;
            case 0x08: // right button
                turn_flag = 1;
                t = 1.0;
                break;
            }
        }

        // If commutator state requires change
        if (context_flag > 0)
        { 
            context_flag = 0;
            update_context(ctx, mot_ctx);
            write_context_flag = 1;
        }

        // If context needs to be written to flash
        if (write_context_flag > 0 && !mot_ctx.motor.isRunning())
        { 
            write_context_flag = 0;
            write_context_to_flash(ctx);
        }

        // If turn or enable cmd/button press is detected
        if (turn_flag > 0)
        { 
            turn_flag = 0;
            turn_cmd(ctx, mot_ctx, t);
        }
    }
    return 0;
}

void print_context(Context context, MotorContext motor_context)
{
    JsonDocument print_doc;
#ifdef DEBUG
    printf("print\n");
#endif
    print_doc["type"] = COMMUTATOR_TYPE;
    print_doc["board_rev"] = BOARD_REV;
    print_doc["firmware"] = FIRMWARE_VER;
    print_doc["enable"] = context.enable;
    print_doc["led"] = context.led;
    print_doc["steps_to_go"] = motor_context.motor.distanceToGo();
    print_doc["target_steps"] = motor_context.motor.targetPosition();
    print_doc["target_turns"] = motor_context.target_turns;
    print_doc["max_turns"] = MAX_TURNS;
    print_doc["motor_running"] = motor_context.motor.distanceToGo() != 0;
    print_doc["charge_current"] = ltc4425_read_charge_current();
    print_doc["power_good"] = ltc4425_power_good();
    std::string print_string;
    serializeJson(print_doc, print_string);
    printf("%s\n", print_string.c_str());
}

void turn_cmd(Context context, MotorContext motor_context, double t)
{
    if (context.enable)
    {
        if (std::isnan(t))
        {
            printf("Turn command was NaN\n");
        }
        else if (std::isinf(t))
        {
            printf("Turn command was Inf\n");
        }
        else
        {
            motor_turn(motor_context, t);
#ifdef DEBUG
            printf("Motor turning\n");
#endif
        }
    }
    else if (!context.enable)
    {
        printf("Cannot move when commutator is disabled\n");
    }
}

void write_context_to_flash(Context context)
{
#ifdef DEBUG
    printf("Writing context to flash...\n");
#endif
    // The two flash functions are unsafe
    // XIP instructions in the other core and interrupts must be disabled
    uint32_t interrupts = save_and_disable_interrupts();
    multicore_lockout_start_blocking();
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_TARGET_OFFSET, (const uint8_t *)&context, FLASH_PAGE_SIZE);
    multicore_lockout_end_blocking();
    restore_interrupts(interrupts);
#ifdef DEBUG
    printf("Context written to flash\n");
#endif
}

void update_context(Context context, MotorContext motor_context)
{
    motor_enable(context.enable, motor_context);
    rgb_update(context);
    // /////////////////////////////////////////////////////////////////////////////
    // #ifdef DEBUG
    //             printf("Writing context to flash...\n");
    // #endif
    //             interrupts = save_and_disable_interrupts();
    //             flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    //             flash_range_program(FLASH_TARGET_OFFSET, (const uint8_t *)&ctx, FLASH_PAGE_SIZE);
    //             restore_interrupts(interrupts);
    // #ifdef DEBUG
    //             printf("Context written to flash\n");
    // #endif
    // /////////////////////////////////////////////////////////////////////////////
}

void print_diagnostics()
{
    uint8_t buffer[1];
    sleep_ms(15);
    const uint8_t cap1296_device_id_register = 0xfd;
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, &cap1296_device_id_register, 1, true);
    i2c_read_blocking(I2C_PORT, CAP1296_ADDR, buffer, 1, false);
    sleep_ms(500);
    printf("device id: %02hhx\n", buffer[0]);
    const uint8_t cap1296_control_register = 0x00;
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, &cap1296_control_register, 1, true);
    i2c_read_blocking(I2C_PORT, CAP1296_ADDR, buffer, 1, false);
    printf("control: %02hhx\n", buffer[0]);
    const uint8_t cap1296_status_register = 0x02;
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, &cap1296_status_register, 1, true);
    i2c_read_blocking(I2C_PORT, CAP1296_ADDR, buffer, 1, false);
    printf("status: %02hhx\n", buffer[0]);
    const uint8_t cap1296_calibration_status_register = 0x26;
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, &cap1296_calibration_status_register, 1, true);
    i2c_read_blocking(I2C_PORT, CAP1296_ADDR, buffer, 1, false);
    printf("calibration: %02hhx\n", buffer[0]);
    const uint8_t cap1296_noise_register = 0x0a;
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, &cap1296_noise_register, 1, true);
    i2c_read_blocking(I2C_PORT, CAP1296_ADDR, buffer, 1, false);
    printf("noise: %02hhx\n", buffer[0]);
    const uint8_t cap1296_cal1_register = 0xb1;
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, &cap1296_cal1_register, 1, true);
    i2c_read_blocking(I2C_PORT, CAP1296_ADDR, buffer, 1, false);
    printf("cal1: %02hhx\n", buffer[0]);
    const uint8_t cap1296_cal2_register = 0xb2;
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, &cap1296_cal2_register, 1, true);
    i2c_read_blocking(I2C_PORT, CAP1296_ADDR, buffer, 1, false);
    printf("cal2: %02hhx\n", buffer[0]);
    const uint8_t cap1296_cal3_register = 0xb3;
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, &cap1296_cal3_register, 1, true);
    i2c_read_blocking(I2C_PORT, CAP1296_ADDR, buffer, 1, false);
    printf("cal3: %02hhx\n", buffer[0]);
    const uint8_t cap1296_cal4_register = 0xb4;
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, &cap1296_cal4_register, 1, true);
    i2c_read_blocking(I2C_PORT, CAP1296_ADDR, buffer, 1, false);
    printf("cal4: %02hhx\n", buffer[0]);
    const uint8_t cap1296_lsb1_register = 0xb9;
    i2c_write_blocking(I2C_PORT, CAP1296_ADDR, &cap1296_lsb1_register, 1, true);
    i2c_read_blocking(I2C_PORT, CAP1296_ADDR, buffer, 1, false);
    printf("lsb1: %02hhx\n", buffer[0]);
}