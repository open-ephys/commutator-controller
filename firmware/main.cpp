#include <ArduinoJson.h>
#include <climits>
#include <cmath>
#include <ios>
#include <iostream>
#include <stdio.h>
#include <tusb.h>
#include <istream>

#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico/util/queue.h"

#include "AccelStepper.h"
#include "cap1296.h"
#include "io.h"
#include "is32fl3193.h"
#include "ltc4425.h"
#include "rotor.h"
#include "pindefs.h"

// #define DEBUG
#define FIRMWARE_VER "0.1.0"
#define BOARD_REV "G"

#ifdef DEBUG
    uint16_t button_counter = 0;
#endif

#define MAX_SERIAL_BUFFER_LENGTH 1024u

// ISR flag for capacitative touch detected
volatile bool alert_flag = false;
static void io_alert_irq_callback(unsigned int gpio, long unsigned int events)
{
    alert_flag = true;
};

// Shared state between cores: thread-safe queues
queue_t rotor_cmd_queue;

// NB: The gear ratio is configurable without recompilation using
//      `picotool config -s gear_ratio 3.14`
// or similar.
double gear_ratio = 2.0;
bi_decl(bi_ptr_string(0, 0, gear_ratio_str, "2.0", 32));

struct Context {
    bool enable = false;
    bool led = true;
    uint8_t last_sensor_input_status = BUTTON_RELEASE;
} ctx;

enum class rotor_cmd_tag {ENABLE, TURN, STOP};

struct rotor_cmd_t {
    rotor_cmd_tag tag;
    union {
        bool enable;
        double turns;
    } value;
};

static void turn_command(Context *ctx, double turns)
{
    if (ctx->enable)
    {
        rotor_cmd_t rotor_cmd = {.tag = rotor_cmd_tag::TURN, .value = {.turns = turns}};
        queue_add_blocking(&rotor_cmd_queue, &rotor_cmd);
    }
    else
    {
        JsonDocument doc;
        doc["error"] = "Turn command received while disabled";
        serializeJson(doc, std::cout);
        std::cout << std::endl;
    }
}

static void turn_button(bool direction, bool enable)
{
    if (enable)
    {
        double turns = direction ? -100 : 100;
        const rotor_cmd_t rotor_cmd_stop = {.tag = rotor_cmd_tag::STOP};
        const rotor_cmd_t rotor_cmd_turn = {.tag = rotor_cmd_tag::TURN, .value = {.turns = turns}};
        queue_add_blocking(&rotor_cmd_queue, &rotor_cmd_stop);
        queue_add_blocking(&rotor_cmd_queue, &rotor_cmd_turn);
    }
}

static void process_button_touches(Context *ctx)
{
    if (alert_flag) {
        rotor_cmd_t rotor_cmd;
        cap1296_clear_int_bit_in_main_control_register();
        uint8_t sensor_input_status = cap1296_read_sensor_input_status_register();
        switch (sensor_input_status)
        {
            case LED_BUTTON_PRESS:
                ctx->led = !ctx->led;
                rgb_set_auto(ctx->enable, ctx->led);
                break;
            case ENABLE_BUTTON_PRESS:
                ctx->enable = !ctx->enable;
                rotor_cmd = {.tag = rotor_cmd_tag::ENABLE, .value = {.enable = ctx->enable}};
                queue_add_blocking(&rotor_cmd_queue, &rotor_cmd);
                rgb_set_auto(ctx->enable, ctx->led);
                break;
            case CW_BUTTON_PRESS:
                turn_button(false, ctx->enable);
                break;
            case CCW_BUTTON_PRESS:
                turn_button(true, ctx->enable);
                break;
            case BUTTON_RELEASE:
                if (ctx->last_sensor_input_status & (CW_BUTTON_PRESS | CCW_BUTTON_PRESS))
                {
                    rotor_cmd.tag = rotor_cmd_tag::STOP;
                    queue_add_blocking(&rotor_cmd_queue, &rotor_cmd);
                }
                break;
        }
        ctx->last_sensor_input_status = sensor_input_status;
        alert_flag = false;
#ifdef DEBUG
        printf("{button: %02hhx, counter: %i}\n", sensor_input_status, button_counter++);
#endif
    }
}

static void process_serial_commands(Context *ctx)
{
    char serial_buffer[MAX_SERIAL_BUFFER_LENGTH] = {0};
    std::cin.getline(serial_buffer, MAX_SERIAL_BUFFER_LENGTH);
    if (std::cin.rdstate() == std::ios_base::failbit)
    {
        JsonDocument doc;
        char buf[256];
        snprintf(buf, sizeof(buf), "Command must be less than %d bytes long.", MAX_SERIAL_BUFFER_LENGTH);
        doc["error"] = buf;
        serializeJson(doc, std::cout);
        std::cout << std::endl;

        std::cin.clear();
        std::cin.ignore(INT_MAX, '\n');
        return;
    }

    JsonDocument receive;
    auto error = deserializeJson(receive, serial_buffer, MAX_SERIAL_BUFFER_LENGTH);

    if (error.code() != DeserializationError::Ok)
    {
        JsonDocument doc;
        doc["error"] = "Bad JSON formatting";
        serializeJson(doc, std::cout);
        std::cout << std::endl;
        return;
    }

    rotor_cmd_t rotor_cmd;

    // Enable command
    if (receive["enable"].is<bool>())
    {
        ctx->enable = receive["enable"];
        rotor_cmd = {.tag = rotor_cmd_tag::ENABLE, .value = {.enable = ctx->enable}};
        queue_add_blocking(&rotor_cmd_queue, &rotor_cmd);
        rgb_set_auto(ctx->enable, ctx->led);
    }

    // LED command
    if (receive["led"].is<bool>())
    {
        ctx->led = receive["led"];
        rgb_set_auto(ctx->enable, ctx->led);
    }

    // Turn command, but don't let this command override current button presses
    if (receive["turn"].is<double>())
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
           turn_command(ctx, turns);
        }
    }

    // Print command
    if (receive["print"].is<JsonVariant>())
    {
        JsonDocument doc;
        doc["gear_ratio"] = gear_ratio;
        doc["board_rev"] = BOARD_REV;
        doc["firmware"] = FIRMWARE_VER;
        doc["enable"] = ctx->enable;
        doc["led"] = ctx->led;
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

static void core1_entry()
{
    rotor_t rotor = { AccelStepper(AccelStepper::DRIVER, TMC2130_STEP, TMC2130_DIR), atof(gear_ratio_str)};
    rotor_cmd_t rotor_cmd;
    double turn_command;
    bool enable_command;
    bool stop_flag;
    rotor_init(&rotor);
    while (true)
    {
        if (queue_try_remove(&rotor_cmd_queue, &rotor_cmd))
        {
            switch (rotor_cmd.tag)
            {
                case rotor_cmd_tag::ENABLE:
                    rotor_enable(&rotor, rotor_cmd.value.enable);
                    break;
                case rotor_cmd_tag::TURN:
                    rotor_move(&rotor, rotor_cmd.value.turns);
                    break;
                case rotor_cmd_tag::STOP:
                    rotor_stop(&rotor);
                    break;
                default:
                    rotor_try_to_zero_position(&rotor);
                    break;
            }
        }
        else
        {
            rotor_try_to_zero_position(&rotor);
        }
        rotor.motor.run();
    }
}

int main()
{
    gear_ratio = atof(gear_ratio_str);
    queue_init(&rotor_cmd_queue, sizeof(rotor_cmd_t), 32);

    // Initialize serial port
    stdio_init_all();

    // Initialize chips
    io_init();
    rgb_init();
    rgb_set_auto(ctx.enable, ctx.led);
    rgb_set_breathing(true);
    ltc4425_init();
    cap1296_init();
    while (!ltc4425_power_good()) { tight_loop_contents(); } // Wait for supercaps to charge
    rgb_set_breathing(false);
    rgb_set_auto(ctx.enable, ctx.led);

    // Launch motor driver loop
    multicore_launch_core1(core1_entry); // Launch core1

    // Enable button alert ISR and clear any button touches registered during initialization
    gpio_set_irq_enabled_with_callback(CAP1296_ALERT, GPIO_IRQ_EDGE_FALL, true, &io_alert_irq_callback);
    cap1296_clear_int_bit_in_main_control_register();

    // Decode commands and buttons
    while (true)
    {
        process_button_touches(&ctx);

        if (ctx.last_sensor_input_status == BUTTON_RELEASE && tud_cdc_available()) {
            process_serial_commands(&ctx);
        }
    }

    return 0;
}

