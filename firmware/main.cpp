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
#define BOARD_REV "G"

#ifdef DEBUG
    uint16_t button_counter = 0;
#endif

#define MAX_GEAR_RATIO 100
#define MAX_SERIAL_BUFFER_LENGTH 1024u

// ISR flag for capacitative touch detected
volatile bool alert_flag = false;
static void io_alert_irq_callback(unsigned int gpio, long unsigned int events)
{
    alert_flag = true;
};

// NB: The gear ratio is configurable without recompilation using
//      `picotool config -s gear_ratio 3.14`
// or similar.
double gear_ratio_f = 2.0;
bi_decl(bi_ptr_string(0, 0, gear_ratio, "2.0", 32));

struct context_t {
    bool enable = false;
    bool led = true;
    uint8_t last_sensor_input_status = BUTTON_RELEASE;
};

// Thread-safe queue and command types that are shared state between cores
queue_t rotor_cmd_queue;

enum class rotor_cmd_tag {ENABLE, TURN, STOP};

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
    ERROR_INVALID_TURN_REQ = -1,
    ERROR_COMMAND_BUFFER_OVERFLOW = -2,
    ERROR_COMMAND_INCOMPLETE = -3,
    ERROR_MALFORMED_JSON = -4,
    ERROR_NAN_OR_INF_TURN_COMMAND = -5,
    ERROR_REMOTE_INTERFACE_LOCKED = -6
} commutator_error_t;

static const char *error_str(int error)
{
    switch (error)
    {
        case ERROR_SUCCESS:
            return "Success";
        case ERROR_INVALID_TURN_REQ:
            return "Cannot enqueue turn command when commutator is disabled.";
        case ERROR_COMMAND_BUFFER_OVERFLOW:
            return "Command exceeded " xstr(MAX_SERIAL_BUFFER_LENGTH) " byte maximum.";
        case ERROR_COMMAND_INCOMPLETE:
            return "A complete command could not be constructed from the input stream.";
        case ERROR_MALFORMED_JSON:
            return "Malformed JSON received.";
        case ERROR_NAN_OR_INF_TURN_COMMAND:
            return "Turn command with value NaN or Inf received.";
        case ERROR_REMOTE_INTERFACE_LOCKED:
            return "A remote command was received when remote interface was locked";
        default:
            return "Invalid error code.";
    }
}

static int queue_add_turn_cmd_blocking(const context_t *ctx, double turns)
{
    if (ctx->enable)
    {
        rotor_cmd_t rotor_cmd = {.tag = rotor_cmd_tag::TURN, .value = {.turns = turns}};
        queue_add_blocking(&rotor_cmd_queue, &rotor_cmd);
        return ERROR_SUCCESS;
    }

    return ERROR_INVALID_TURN_REQ;
}

static int process_button_touches(context_t *ctx)
{
    int rc = ERROR_SUCCESS;
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
                rotor_cmd = {.tag = rotor_cmd_tag::STOP};
                queue_add_blocking(&rotor_cmd_queue, &rotor_cmd);
                rc = queue_add_turn_cmd_blocking(ctx, -100);
                break;
            case CCW_BUTTON_PRESS:
                rotor_cmd = {.tag = rotor_cmd_tag::STOP};
                queue_add_blocking(&rotor_cmd_queue, &rotor_cmd);
                rc = queue_add_turn_cmd_blocking(ctx, 100);
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

    return rc;
}

static inline bool remote_available(const context_t *ctx)
{
    return ctx->last_sensor_input_status == BUTTON_RELEASE ||
           ctx->last_sensor_input_status == LED_BUTTON_PRESS;
}

static int build_serial_buffer(char *serial_buffer)
{
    static uint16_t serial_buffer_index = 0;
    serial_buffer[serial_buffer_index++] = getchar();
    if (serial_buffer_index >= MAX_SERIAL_BUFFER_LENGTH)
    {
        serial_buffer_index = 0;
        memset(serial_buffer, 0, MAX_SERIAL_BUFFER_LENGTH);
        return ERROR_COMMAND_BUFFER_OVERFLOW;
    }
    else if (serial_buffer[serial_buffer_index - 1] == '}')
    {
        serial_buffer_index = 0;
        return ERROR_SUCCESS;
    }

    return ERROR_COMMAND_INCOMPLETE;
}

static int process_serial_commands(context_t *ctx)
{
    static char serial_buffer[MAX_SERIAL_BUFFER_LENGTH] = {0};

    int rc = build_serial_buffer(serial_buffer);
    if (rc){ return rc; }

    if(!remote_available(ctx))
    {
        return ERROR_REMOTE_INTERFACE_LOCKED;
    }

    JsonDocument receive;
    auto error = deserializeJson(receive, serial_buffer, MAX_SERIAL_BUFFER_LENGTH);
    memset(serial_buffer, 0, MAX_SERIAL_BUFFER_LENGTH);

    if (error.code() != DeserializationError::Ok)
    {
        return ERROR_MALFORMED_JSON;
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
        if (std::isnan(turns) || std::isinf(turns))
        {
            rc = ERROR_NAN_OR_INF_TURN_COMMAND;
        }
        else
        {
            rc = queue_add_turn_cmd_blocking(ctx, turns);
        }
    }

    // Print command
    if (receive["print"].is<JsonVariant>())
    {
        JsonDocument doc;
        doc["gear_ratio"] = gear_ratio_f;
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

    return rc;
}

static void core1_entry()
{
    rotor_t rotor = {AccelStepper(AccelStepper::DRIVER, TMC2130_STEP, TMC2130_DIR), gear_ratio_f, 0.0};
    rotor_cmd_t rotor_cmd;
    rotor_init(&rotor);

    rotor_enable(&rotor, false);

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
                    rotor_stop_and_reset(&rotor);
                    break;
            }
        }

        // If the motor has completed its motion and is stopped, reset the internal position counter
        if (!rotor.motor.run())
        {
            rotor_stop_and_reset(&rotor);
        }
    }
}

int main()
{
    context_t ctx;

    gear_ratio_f = atof(gear_ratio);
    if (gear_ratio_f < -MAX_GEAR_RATIO || gear_ratio_f > MAX_GEAR_RATIO)
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
        int rc = process_button_touches(&ctx);
#ifdef DEBUG
        std::cerr << error_str(rc) << "\n";
#endif

        if (tud_cdc_available()) {
            rc = process_serial_commands(&ctx);
#ifdef DEBUG
            std::cerr << error_str(rc) << "\n";
#endif
        }

#ifdef DEBUG
        printf("%f\n", ltc4425_charge_current());
#endif
    }

    return EXIT_SUCCESS;
}

