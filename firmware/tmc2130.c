#include "pico/stdlib.h"
#include "pindefs.h"
#include "hardware/spi.h"
#include "tmc2130.h"

#define WRITE_FLAG              (1<<7)
#define READ_FLAG               (0<<7)

static inline void cs_select() {
    asm volatile("nop \n nop \n nop");
    gpio_put(TMC2130_CFG3_CS, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(TMC2130_CFG3_CS, 1);
    asm volatile("nop \n nop \n nop");
}

static void tmc2130_write(uint8_t reg, uint32_t data)
{
    uint8_t buf[5] = {WRITE_FLAG | reg, (data >> 24UL) & 0xFF, (data >> 16UL) & 0xFF, (data >> 8UL) & 0xFF, (data >> 0UL) & 0xFF};

    cs_select();
    spi_write_blocking(TMC2130_SPI_PORT, buf, 5);
    cs_deselect();
}

void tmc2130_init()
{
    spi_init(TMC2130_SPI_PORT, 1000 * 1000);
    gpio_set_function(TMC2130_CFG2_SCLK, GPIO_FUNC_SPI);
    gpio_set_function(TMC2130_CFG0_MISO, GPIO_FUNC_SPI);
    gpio_set_function(TMC2130_CFG1_MOSI, GPIO_FUNC_SPI);

    gpio_init(TMC2130_CFG3_CS);
    gpio_set_dir(TMC2130_CFG3_CS, GPIO_OUT);
    gpio_put(TMC2130_CFG3_CS, 1);

    gpio_init(TMC2130_DIR);
    gpio_set_dir(TMC2130_DIR, GPIO_OUT);
    gpio_put(TMC2130_DIR, 0);

    gpio_init(TMC2130_STEP);
    gpio_set_dir(TMC2130_STEP, GPIO_OUT);
    gpio_put(TMC2130_STEP, 0);

    gpio_init(TMC2130_CFG6_EN);
    gpio_set_dir(TMC2130_CFG6_EN, GPIO_OUT);

    // voltage on AIN is current reference
    // Stealthchop is on
    tmc2130_write(REG_GCONF, 0x00000007UL);

    // Configure steathchip
    // PWM_GRAD = 0x0F
    // PWM_AMPL = 0xFF
    // pwm_autoscale = 0x01
    tmc2130_write(REG_PWMCONF, 0x00040FFFUL);

    // IHOLD = 0x0A
    // IRUN = 0x1F (Max)
    // IHOLDDELAY = 0x06
    tmc2130_write(REG_IHOLD_IRUN, 0b01100001111100011111UL); // 0x00_04_1F_UL);

    // 8 microsteps per step
    tmc2130_write(REG_CHOPCONF, 0x05008008UL); 

    // Start in disabled state
    tmc2130_enable(0);
}