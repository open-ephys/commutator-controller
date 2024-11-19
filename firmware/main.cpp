#include <stdio.h>
#include <iostream>
#include <cmath>
#include <tusb.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/flash.h"
#include "hardware/gpio.h"

#include <ArduinoJson.h>

#include "defs.h"
#include "pindefs.h"
#include "ltc4425.h"
#include "is32fl3193.h"
#include "motor.h"
#include "cap1296.h"

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
void serial_input();
uint8_t button_input(uint8_t previous_button_press);
void write_context_to_flash(Context context);
void io_alert_irq_callback(unsigned int gpio, long unsigned int events) { alert_flag = 1; }

// Holds the current state
Context ctx;
MotorContext mot_ctx{.motor = AccelStepper(AccelStepper::DRIVER, TMC2130_STEP, TMC2130_DIR), .target_turns = 0.0};

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
    uint8_t button_press; // for button logic
    // Initialize chips
    stdio_init_all();
    motor_enable(false);
    ctx = *(Context *)(XIP_BASE + FLASH_TARGET_OFFSET); // Read context from flash
    rgb_init(ctx);
    ltc4425_init();

    // Wait for chips to wake up
    sleep_ms(10);

    // Wait for supercaps to charge
    rgb_set_red();
    rgb_set_breathing(true);
    while (!ltc4425_power_good())
    {
        sleep_ms(10);
    }

    rgb_set_breathing(false);
    rgb_set_auto(ctx);

    cap1296_init(); // Capacitative touch sensor

    // Initialize motor
    motor_init(ctx, mot_ctx);

    // Start the second core with motor.run() as its only task
    multicore_launch_core1(core1_entry);
    cap1296_clear_touch_status();
    gpio_set_irq_enabled_with_callback(CAP1296_ALERT, GPIO_IRQ_EDGE_FALL, true, &io_alert_irq_callback);
    // Decode commands and buttons
    while (true)
    {
        // if data is available in serial buffer
        if (tud_cdc_available())
        {
            serial_input();
        }
        // if a button is pressed or released
        if (alert_flag > 0)
        {
            alert_flag = 0;
            button_press = button_input(button_press);
        }
    }
    return 0;
}

void serial_input()
{
    JsonDocument receive;
    auto error = deserializeJson(receive, std::cin);
    if (error)
    {
        return;
    }

    auto enable = receive["enable"];
    auto turns = receive["turn"];
    auto led = receive["led"];
    auto print = receive["print"];

    // Enable command
    if (enable.is<bool>())
    {
        ctx.enable = enable;
        // write_context_to_flash(ctx);
        if (!ctx.enable)
        {
            motor_hard_stop(mot_ctx);
        }
        else
        {
            motor_enable(true);
        }
#ifdef DEBUG
        printf("{enable: %d}\n", ctx.enable);
#endif
        rgb_set_auto(ctx);
    }

    // LED command
    if (led.is<bool>())
    {
        ctx.led = led;
        // write_context_to_flash(ctx);
#ifdef DEBUG
        printf("{led: %d}\n", ctx.led);
#endif
        rgb_set_auto(ctx);
    }

    // Turn command
    if (turns.is<double>())
    {
        double t = turns.as<double>();
        if (!ctx.enable)
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
#ifdef DEBUG
        printf("{turn: %g}\n", t);
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

uint8_t button_input(uint8_t previous_button_press)
{
    uint8_t touch_status_register = cap1296_read_touch_status();
    cap1296_clear_touch_status();
    uint8_t button_press = extract_touch_data(previous_button_press, touch_status_register);
#ifdef DEBUG
    printf("State: %02hhx, Button: %02hhx\n", touch_status_register, button_press);
#endif
    switch (button_press)
    {
    case 0x04: // top button
        ctx.led = !ctx.led;
        // write_context_to_flash(ctx);
        rgb_set_auto(ctx);
        break;
    case 0x02: // bottom button
        ctx.enable = !ctx.enable;
        // write_context_to_flash(ctx);
        rgb_set_auto(ctx);
        break;
    case 0x01: // left button
        motor_turn(mot_ctx, 1);
        break;
    case 0x08: // right button
        motor_turn(mot_ctx, -1);
        break;
    }
    return button_press;
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