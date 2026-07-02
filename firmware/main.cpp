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

#define xstr(a) str(a)
#define str(a) #a

// #define DEBUG
#define FIRMWARE_VER "0.1.0"
#define BOARD_REV "H"

#ifdef DEBUG
    uint16_t button_counter = 0;
#endif

#define MAX_GEAR_RATIO 100
#define MAX_SERIAL_BUFFER_LENGTH 1024u
#define BUTTON_HOLD_TURNS 200

// ISR flag for capacitative touch detected
volatile bool alert_flag = false;
static void io_alert_irq_callback(unsigned int gpio, long unsigned int events)
{
    alert_flag = true;
}

// NB: The gear ratio is configurable without recompilation using
//      `picotool config -s gear_ratio 3.14`
// or similar.
double gear_ratio_f = 2.0;
bi_decl(bi_ptr_string(0, 0, gear_ratio, "2.0", 32));

struct context_t {
    bool enable = false;
    bool led = true;
    bool power_good = false;
    uint8_t last_sensor_input_status = BUTTON_RELEASE;
};

// Thread-safe queue and command types that are shared state between cores
queue_t rotor_cmd_queue;

enum class rotor_cmd_tag {ENABLE, TURN, BUTTON_TURN, STOP};

struct rotor_cmd_t {
    rotor_cmd_tag tag;
    union {
        bool enable;
        double turns;
    } value;
};

// Errors
typedef enum {
    ERROR_SUCCESS = 0,
    ERROR_INVALID_TURN_REQ = 1,
    ERROR_COMMAND_BUFFER_OVERFLOW = 2,
    ERROR_COMMAND_MALFORMED = 4,
    ERROR_NAN_TURN_COMMAND = 8,
    ERROR_REMOTE_INTERFACE_LOCKED = 16,
    ERROR_POWER_BAD = 32
} commutator_error_t;

static void print_report(context_t *ctx, int error){
    JsonDocument doc;
    doc["gear_ratio"] = gear_ratio_f;
    doc["board_rev"] = BOARD_REV;
    doc["firmware"] = FIRMWARE_VER;
    doc["enable"] = ctx->enable;
    doc["led"] = ctx->led;
    doc["charge_current"] = ltc4425_charge_current();
    doc["power_good"] = ltc4425_power_good();
    doc["error"] = error;
    serializeJson(doc, std::cout);
    std::cout << std::endl;
}

static inline bool remote_motor_control_available(const context_t *ctx)
{
    return ctx->last_sensor_input_status == BUTTON_RELEASE ||
           ctx->last_sensor_input_status == LED_BUTTON_PRESS;
}

static int queue_add_cmd_blocking(const context_t *ctx, const rotor_cmd_t *cmd)
{
    int rc = ERROR_SUCCESS;

    switch (cmd->tag)
    {
        case rotor_cmd_tag::ENABLE:
            if (!ctx->power_good)
                rc |= ERROR_POWER_BAD;
            break;

        case rotor_cmd_tag::BUTTON_TURN:

            if (!ctx->power_good)
                rc |= ERROR_POWER_BAD;
            if (!ctx->enable)
                rc |= ERROR_INVALID_TURN_REQ;
            if (std::isnan(cmd->value.turns))
                rc |= ERROR_NAN_TURN_COMMAND;
            break;

        case rotor_cmd_tag::TURN:
            if (!ctx->power_good)
                rc |= ERROR_POWER_BAD;
            if (std::isnan(cmd->value.turns))
                rc |= ERROR_NAN_TURN_COMMAND;
            if (!remote_motor_control_available(ctx))
                rc |= ERROR_REMOTE_INTERFACE_LOCKED;
            if (!ctx->enable)
                rc |= ERROR_INVALID_TURN_REQ;
            break;
    }

    if (rc) return rc;

    queue_add_blocking(&rotor_cmd_queue, cmd);
    return ERROR_SUCCESS;
}

static int process_button_touches(context_t *ctx)
{
    int rc = ERROR_SUCCESS;
    if (alert_flag) {

        alert_flag = false;
        cap1296_clear_int_bit_in_main_control_register();
        uint8_t sensor_input_status = cap1296_read_sensor_input_status_register();

        switch (sensor_input_status)
        {
            case LED_BUTTON_PRESS:
                ctx->led = !ctx->led;
                rgb_set_auto(ctx->enable, ctx->led);
                break;
            case ENABLE_BUTTON_PRESS:
            {
                bool temp = !ctx->enable;
                rotor_cmd_t rotor_cmd = {.tag = rotor_cmd_tag::ENABLE, .value = {.enable = temp}};
                rc |= queue_add_cmd_blocking(ctx, &rotor_cmd);
                if (!rc) ctx->enable = temp;
                rgb_set_auto(ctx->enable, ctx->led);
                break;
            }
            case CW_BUTTON_PRESS:
            {
                rotor_cmd_t rotor_cmd = {.tag = rotor_cmd_tag::STOP};
                rc |= queue_add_cmd_blocking(ctx, &rotor_cmd);
                rotor_cmd = {.tag = rotor_cmd_tag::BUTTON_TURN, .value = {.turns = -BUTTON_HOLD_TURNS}};
                rc |= queue_add_cmd_blocking(ctx, &rotor_cmd);
                break;
            }
            case CCW_BUTTON_PRESS:
            {
                rotor_cmd_t rotor_cmd = {.tag = rotor_cmd_tag::STOP};
                rc |= queue_add_cmd_blocking(ctx, &rotor_cmd);
                rotor_cmd = {.tag = rotor_cmd_tag::BUTTON_TURN, .value = {.turns = BUTTON_HOLD_TURNS}};
                rc |= queue_add_cmd_blocking(ctx, &rotor_cmd);
                break;
            }
            case BUTTON_RELEASE:
            {
                if (ctx->last_sensor_input_status & (CW_BUTTON_PRESS | CCW_BUTTON_PRESS))
                {
                    rotor_cmd_t rotor_cmd = {.tag = rotor_cmd_tag::STOP};
                    rc |= queue_add_cmd_blocking(ctx, &rotor_cmd);
                }
                break;
            }
        }

        ctx->last_sensor_input_status = sensor_input_status;


#ifdef DEBUG
        printf("{button: %02hhx, counter: %i}\n", sensor_input_status, button_counter++);
#endif
    }

    return rc;
}

static int parse_json_command(context_t *ctx, char *buffer)
{
    JsonDocument receive;
    auto error = deserializeJson(receive, buffer);
    if (error.code() != DeserializationError::Ok)
    {
        return ERROR_COMMAND_MALFORMED;
    }

    int rc = ERROR_SUCCESS;

    // Enable command
    if (receive["enable"].is<bool>())
    {
        bool temp = receive["enable"];
        rotor_cmd_t rotor_cmd = {.tag = rotor_cmd_tag::ENABLE, .value = {.enable = temp}};
        rc |= queue_add_cmd_blocking(ctx, &rotor_cmd);
        if (!rc) ctx->enable = temp;
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
        rotor_cmd_t rotor_cmd = {.tag = rotor_cmd_tag::TURN, .value = {.turns = receive["turn"]}};
        rc |= queue_add_cmd_blocking(ctx, &rotor_cmd);
    }

    // Print command
    if (receive["print"].is<JsonVariant>())
    {
        print_report(ctx, ERROR_SUCCESS);
    }

#ifdef DEBUG
    serializeJson(receive, std::cout);
    std::cout << std::endl;
#endif

    return rc;
}

static int process_serial_input(context_t *ctx)
{
    static char serial_buffer[MAX_SERIAL_BUFFER_LENGTH] = {0};
    static uint16_t serial_buffer_index = 0;

    while (tud_cdc_available()) {

        // overflow guard
        if (serial_buffer_index >= MAX_SERIAL_BUFFER_LENGTH - 1) {
            serial_buffer_index = 0;
            memset(serial_buffer, 0, MAX_SERIAL_BUFFER_LENGTH);
            return ERROR_COMMAND_BUFFER_OVERFLOW;
        }

        char c = getchar_timeout_us(0);

        if (c == '{')
        {
            // force index 0 and start over
            memset(serial_buffer, 0, serial_buffer_index + 1);
            serial_buffer_index = 0;
            serial_buffer[serial_buffer_index++] = c;

        }
        else if (c == '}')
        {
            // continue downstairs with current buffer
            serial_buffer[serial_buffer_index] = c;
            int rc = parse_json_command(ctx, serial_buffer);

            memset(serial_buffer, 0, serial_buffer_index + 1);
            serial_buffer_index = 0;

            return rc;
        }
        else
        {
            serial_buffer[serial_buffer_index++] = c;
        }
    }

    return ERROR_SUCCESS;

}

static void core1_entry()
{
    rotor_t rotor = {AccelStepper(AccelStepper::DRIVER, TMC2130_STEP, TMC2130_DIR), gear_ratio_f, 0.0};
    rotor_cmd_t rotor_cmd;
    rotor_init(&rotor);
    static bool await_stop = false;

    rotor_enable(&rotor, false);

    while (true)
    {
        if (!await_stop && queue_try_remove(&rotor_cmd_queue, &rotor_cmd))
        {
            switch (rotor_cmd.tag)
            {
                case rotor_cmd_tag::ENABLE:
                    rotor_enable(&rotor, rotor_cmd.value.enable);
                    break;
                case rotor_cmd_tag::TURN:
                    rotor_set_nomimal_accel(&rotor);
                    rotor_move(&rotor, rotor_cmd.value.turns);
                    break;
                case rotor_cmd_tag::BUTTON_TURN:
                    rotor_set_fast_accel(&rotor);
                    rotor_move(&rotor, rotor_cmd.value.turns);
                    break;
                case rotor_cmd_tag::STOP:
                    rotor_set_fast_accel(&rotor);
                    rotor_stop(&rotor);
                    await_stop = true;
                    break;
            }
        }

        // If the motor has completed its motion and is stopped, reset the internal position counter
        if (!rotor_run(&rotor))
        {
            await_stop = false;
            rotor_stop_and_reset(&rotor);
        }
    }
}

static int check_power(context_t *ctx)
{
    bool pg = ltc4425_power_good();
    bool power_lost = ctx->power_good && !pg;
    bool power_restored = !ctx->power_good && pg;

    int rc = ERROR_SUCCESS;
    if (power_lost)
    {
        ctx->enable = false;
        rotor_cmd_t rotor_cmd = {.tag = rotor_cmd_tag::ENABLE, .value = {.enable = ctx->enable}};
        queue_add_blocking(&rotor_cmd_queue, &rotor_cmd);
        rgb_set_auto(ctx->enable, ctx->led);
        rgb_set_breathing(true);
        rc |= ERROR_POWER_BAD;
    }
    else if (power_restored)
    {
        rgb_set_breathing(false);
        rgb_set_auto(ctx->enable, ctx->led);
    }

    ctx->power_good = pg;
    return rc;
}

int main()
{
    context_t ctx;

    gear_ratio_f = atof(gear_ratio);
    if (gear_ratio_f < -MAX_GEAR_RATIO || gear_ratio_f > MAX_GEAR_RATIO || gear_ratio_f == 0.0)
    {
        // In all likelihood the string was nonsense
        return EXIT_FAILURE;
    }

    // Initialize serial port
    if (!stdio_init_all())
    {
        return EXIT_FAILURE;
    }

    queue_init(&rotor_cmd_queue, sizeof(rotor_cmd_t), 32);

    // Initialize chips
    io_init();
    rgb_init();
    rgb_set_auto(ctx.enable, ctx.led);
    rgb_set_breathing(true);
    ltc4425_init();
    cap1296_init();
    while (!ltc4425_power_good()) { tight_loop_contents(); } // Wait for super caps to charge
    ctx.power_good = true;
    rgb_set_breathing(false);
    rgb_set_auto(ctx.enable, ctx.led);

    // Launch motor driver loop in core1
    multicore_launch_core1(core1_entry);

    // Enable button alert ISR and clear any button touches registered during initialization
    gpio_set_irq_enabled_with_callback(CAP1296_ALERT, GPIO_IRQ_EDGE_FALL, true, &io_alert_irq_callback);
    cap1296_clear_int_bit_in_main_control_register();

    // Decode commands and buttons
    while (true)
    {
        int rc = ERROR_SUCCESS;

        rc |= process_button_touches(&ctx);
        rc |= process_serial_input(&ctx);
        rc |= check_power(&ctx);

        if (rc) print_report(&ctx, rc);

#ifdef DEBUG
        printf("%f\n", ltc4425_charge_current());
#endif
    }

    return EXIT_SUCCESS;
}

