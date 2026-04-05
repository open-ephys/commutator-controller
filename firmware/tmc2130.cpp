#include <cstring>

#include "pico/stdlib.h"
#include "hardware/spi.h"

#include "tmc2130.h"
#include "rotor.h"
#include "math.h"

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
    uint8_t buf[5] = {(uint8_t)(WRITE_FLAG | reg),
                      (uint8_t)((data >> 24UL) & 0xFF),
                      (uint8_t)((data >> 16UL) & 0xFF),
                      (uint8_t)((data >> 8UL) & 0xFF),
                      (uint8_t)((data >> 0UL) & 0xFF)};


    cs_select();
    spi_write_blocking(TMC2130_SPI_PORT, buf, 5);
    cs_deselect();
}

static uint32_t tmc2130_read(uint8_t reg)
{
    // Refer to 4.1.1 of datasheet for clarification regarding this
    // implementation
    uint8_t rx[5];
    uint8_t tx[5] = {(uint8_t)(READ_FLAG | reg), 0, 0, 0, 0};

    cs_select();
    spi_write_read_blocking(TMC2130_SPI_PORT, tx, rx, 5);
    cs_deselect();

    sleep_us(1);

    tx[0] = 0;
    cs_select();
    spi_write_read_blocking(TMC2130_SPI_PORT, tx, rx, 5);
    cs_deselect();

    return (uint32_t)rx[1] << 24 | (uint32_t)rx[2] << 16 | (uint32_t)rx[3] << 8  | (uint32_t)rx[4];
}

void tmc2130_init()
{
    gpio_put(TMC2130_CFG3_CS, 1);
    gpio_put(TMC2130_DIR, 0);
    gpio_put(TMC2130_STEP, 0);

    // Page 25: General Configuration
    // - en_pwm_mode = 0b1 (enable SteathChop)
    // - others - 0b0
    tmc2130_write(REG_GCONF, 0x00000004);

    // Page 34: Chopper configuration
    // - TOFF = 3
    // - HSTRT = 4
    // - HEND = 1
    // - CHM = 0 (SpreadCycle)
    // - TBL = 2
    // - VSENSE = 1
    // - MRES = log2(256/USTEPS_PER_STEP)
    // - INTPOL = 0
    uint32_t ustep_setting = (USTEPS_PER_STEP == 256 ? 0x00 : (int32_t)log2(256 / USTEPS_PER_STEP)) << 24 ;
    tmc2130_write(REG_CHOPCONF, 0x000300C3 | ustep_setting);

    // Page 27: Current configuration
    // * RSENSE = 0.47 Ohms
    // * VSENSE = 1
    // * I_RMS_MAX = 0.26A (Page 55)
    // - IHOLD = 0x1F (I_RMS = 0.26A)
    // - IRUN = 0x1F (I_RMS = 0.26A)
    tmc2130_write(REG_IHOLD_IRUN, 0x00061F1F);

    // Page 37: SteathChop Configuration
    // PWM_AMPL = 0xC8 (not used though since we stay in SteathChop mode)
    // freewheel = 0b00 (Normal)
    // pwm_symmetric = 0b0
    // pwm_autoscale = 0b1
    // pwm_freq = 0b01 (38 kHz, see  Table 6.1, using internal clk)
    tmc2130_write(REG_PWMCONF, 0x000501C8);

    // Start in disabled state
    tmc2130_enable(false);
}

uint32_t tmc2130_status()
{
    return tmc2130_read(REG_DRVSTATUS);
}