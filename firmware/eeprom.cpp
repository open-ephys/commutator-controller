#include "eeprom.h"

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "pico/binary_info.h"  // bi_decl, bi_ptr_int32
#include "pico/stdlib.h"       // XIP_BASE, PICO_FLASH_SIZE_BYTES

// Identifies a provisioned config region. Must match the magic written by
// flash-utility.ps1.
#define EEPROM_MAGIC 0xDEADBEEFu

// The reserved config sector begins exactly where the declared flash ends.
// PICO_FLASH_SIZE_BYTES is set in boards/commutator.h (the single source of
// truth for this board's flash layout) one sector smaller than the physical
// chip, specifically so the linker refuses to place firmware here.
#define EEPROM_CONFIG_ADDR (XIP_BASE + PICO_FLASH_SIZE_BYTES)

// Exposes the address via binary_info so flash-utility.ps1 can read it
// directly from the built ELF (via `picotool config`) instead of re-deriving
// or guessing it.
bi_decl(bi_ptr_int32(0, 0, eeprom_config_addr, EEPROM_CONFIG_ADDR));

// Field width (including the terminating NUL) of the stored gear ratio string.
#define EEPROM_GEAR_RATIO_MAX_LEN 21

// On-flash layout. All fields are byte-aligned (the gear ratio is a decimal
// string, not a binary double) so there is no padding between fields. The
// static_asserts below pin the offsets that flash-utility.ps1 reproduces.
struct eeprom_config_t {
    uint32_t magic;
    char board_rev;
    char gear_ratio[EEPROM_GEAR_RATIO_MAX_LEN];
};

static_assert(offsetof(eeprom_config_t, board_rev) == 4, "unexpected board_rev offset");
static_assert(offsetof(eeprom_config_t, gear_ratio) == 5, "unexpected gear_ratio offset");

// Returns the config region if it carries a valid magic word, else nullptr.
// Flash is memory-mapped (XIP), so the region can be read directly.
static const eeprom_config_t *eeprom_valid_config()
{
    const eeprom_config_t *cfg =
        reinterpret_cast<const eeprom_config_t *>(EEPROM_CONFIG_ADDR);
    return cfg->magic == EEPROM_MAGIC ? cfg : nullptr;
}

bool eeprom_read_board_rev(char *out)
{
    if (out == nullptr)
        return false;

    const eeprom_config_t *cfg = eeprom_valid_config();
    if (cfg == nullptr)
        return false;

    *out = cfg->board_rev;
    return true;
}

bool eeprom_read_gear_ratio(double *out)
{
    if (out == nullptr)
        return false;

    const eeprom_config_t *cfg = eeprom_valid_config();

    // Require the stored string to be NUL-terminated within its field before
    // treating it as a C string.
    if (cfg == nullptr ||
        memchr(cfg->gear_ratio, '\0', sizeof(cfg->gear_ratio)) == nullptr)
        return false;

    char *end;
    double value = strtod(cfg->gear_ratio, &end);
    if (end == cfg->gear_ratio || *end != '\0') // not a clean, complete number
        return false;

    *out = value;
    return true;
}
