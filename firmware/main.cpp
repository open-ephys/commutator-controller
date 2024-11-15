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

#define DEBUG

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

volatile uint8_t alert_flag;
Context ctx;
MotorContext mot_ctx{.motor = AccelStepper(AccelStepper::DRIVER, TMC2130_STEP, TMC2130_DIR), .target_turns = 0.0};

void print_diagnostics();

// void __no_inline_not_in_flash_func(core1_entry)()
void core1_entry()
{
    multicore_lockout_victim_init();
    while (true)
    {
        mot_ctx.motor.run();
    }
}

// std::string ignore_until_open_curly()
// {
//     std::string invalid;
//     for (auto next = std::cin.peek(); next != '{' && !std::cin.eof(); next = std::cin.peek())
//     {
//         invalid += std::cin.get();
//     }
//     return invalid;
// }

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
    uint8_t touch_status_register, previous_button_press, current_button_press, context_flag, turn_flag, write_context_flag;
    uint32_t interrupts;
    double t;
    // Initialize hardware
    io_init(); // RP2040 io
    rgb_set_breathing(true);
    rgb_set_red();
    rgb_init();     // (IS3FL3193) RGB LED driver
    ltc4425_init(); // Super capacitor charger
    cap1296_init(); // Capacitative touch sensor
#ifdef DEBUG
    uint8_t button_counter;
    print_diagnostics();
#endif
    // Start the second core with motor.run() as its only task
    rgb_set_breathing(false);
    ctx = *(Context *)(XIP_BASE + FLASH_TARGET_OFFSET); // Read context from flash
    motor_init(ctx, mot_ctx); // (TMC2130) motor driver
    motor_enable(ctx.enable, mot_ctx);
    sleep_ms(10); // Wait for chips to wake up
    rgb_update(ctx);
    multicore_launch_core1(core1_entry);
    cap1296_clear_touch_status();
    // Decode commands and buttons
    while (true)
    {
        if (tud_cdc_available())
        {
            JsonDocument doc;
            auto error = deserializeJson(doc, std::cin);
            if (error.code() == DeserializationError::Ok)
            {
                for (JsonPair kv : doc.as<JsonObject>())
                {
                    if (!strcmp(kv.key().c_str(), "turn"))
                    {
                        if (kv.value().is<double>())
                        {
                            t = kv.value();
                            turn_flag = 1;
#ifdef DEBUG 
                            printf("%s: %g\n", kv.key().c_str(), t);
#endif
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
#ifdef DEBUG 
                            printf("%s: %d\n", kv.key().c_str(), ctx.led);
#endif
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
#ifdef DEBUG 
                            printf("%s: %d\n", kv.key().c_str(), ctx.enable);
#endif
                        }
                        else
                        {
                            printf("Invalid value for \"%s\"\n", kv.key().c_str());
                        }
                        break;
                    }
                    else if (!strcmp(kv.key().c_str(), "print"))
                    {
                        if (doc["print"].is<JsonVariant>())
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
                            doc["charge_current"] = ltc4425_read_charge_current();
                            doc["power_good"] = ltc4425_power_good();
                            serializeJson(doc, std::cout);
                        }
                    }
                    else
                    {
                        printf("Invalid key: \"%s\"\n", kv.key().c_str());
                    }
                }
            }
            else if (error.code() == DeserializationError::InvalidInput)
            {
                JsonDocument error_doc;
                error_doc["error"] = error.c_str();
                // doc["invalid_str"] = invalid.c_str();
                // serializeJson(error_doc, std::cout);
                printf("Invalid JSON\n");
            }
        }

        if (alert_flag == 1)
        {
            alert_flag = 0;
            touch_status_register = cap1296_read_touch_status();
            cap1296_clear_touch_status();
            current_button_press = extract_touch_data(previous_button_press, touch_status_register);
            previous_button_press = current_button_press;
#ifdef DEBUG
            printf("state: %02hhx, button: %02hhx, count: %u\n", touch_status_register, current_button_press, ++button_counter);
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

        if (context_flag > 0)
        {   // If context was updated
            context_flag = 0;
            motor_enable(ctx.enable, mot_ctx);
            rgb_update(ctx);
            write_context_flag = 1;
        }

        if (write_context_flag > 0 && !mot_ctx.motor.isRunning())
        {   // If context needs to be written to flash
            // The two flash functions are unsafe
            // XIP instructions in the other core and interrupts must be disabled 
            write_context_flag = 0;
            interrupts = save_and_disable_interrupts(); 
            multicore_lockout_start_blocking();
            flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
            flash_range_program(FLASH_TARGET_OFFSET, (const uint8_t *)&ctx, FLASH_PAGE_SIZE);
            multicore_lockout_end_blocking();
            restore_interrupts(interrupts);
#ifdef DEBUG
            printf("context written to flash\n");
#endif
        }

        if (turn_flag > 0)
        {   // if turn or enable cmd/button press is detected
            turn_flag = 0;
            if (ctx.enable)
            {
                if (std::isnan(t))
                {
                    JsonDocument doc;
                    doc["error"] = "Turn command was NaN";
                    serializeJson(doc, std::cout);
                }
                else if (std::isinf(t))
                {
                    JsonDocument doc;
                    doc["error"] = "Turn command was Inf";
                    serializeJson(doc, std::cout);
                }
                else
                {
                    motor_turn(mot_ctx, t);
                }
            }
            else if (!ctx.enable)
            {
                JsonDocument doc;
                doc["error"] = "Cannot move when commutator is disabled";
                serializeJson(doc, std::cout);
            }
        }
    }
    return 0;
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