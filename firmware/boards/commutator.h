// Custom board definition for the Open Ephys Commutator Controller.
//
// Identical to the stock "pico" board (RP2040), except that the declared
// usable flash size is one sector smaller. That last sector holds the
// persistent board_rev/gear_ratio config (see eeprom.cpp). Firmware is
// written from the start of flash, so the smaller declared size keeps an
// image from ever reaching the config sector, so the config survives
// future firmware flashes.
//
// This is enforced by the linker. An image that would overwrite the reserved
// sector fails to link outright ("region 'FLASH' overflowed").

#ifndef _BOARDS_COMMUTATOR_H
#define _BOARDS_COMMUTATOR_H

// pico_cmake_set_default PICO_FLASH_SIZE_BYTES = ((2 * 1024 * 1024) - (1 << 12))
#define PICO_FLASH_SIZE_BYTES ((2 * 1024 * 1024) - (1 << 12))

#include "boards/pico.h"

#endif
