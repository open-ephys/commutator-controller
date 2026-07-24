# Commutator Controller Firmware

Firmware for the RP2040-based Open Ephys Commutator Controller. 

For the commutator to function, the controller must be flashed with board_rev
and gear_ratio values in addition to the firmware. This allows users to update
the firmware while persisting gear ratio and hardware revision information.

`flash-utility.ps1` (run via `flash-utility.cmd`) allows flashing a firmware
image as well as the hardware configuration in one step for use during
production. This document describes how to use it.

## Build Firmware

1. Run `git submodule update --init --recursive` to download the necessary
   submodules. 
2. Follow the [Pico Getting Started Guide](https://pip-assets.raspberrypi.com/categories/610-raspberry-pi-pico/documents/RP-008276-DS-2-getting-started-with-pico.pdf)
   to build the firmware. 

The build process produces `build/commutator.elf`. `flash-utility.ps1` takes
this file's path as input.

The board definition (`boards/commutator.h`) declares one flash sector less
than the physical chip, so the linker itself refuses to link a firmware image
that would overwrite the reserved configuration sector described below (a
build failure, `region 'FLASH' overflowed`).

## Flash Firmware and Hardware Configuration

Requires [`picotool`](https://github.com/raspberrypi/picotool) **2.1.0 or
later** on your `PATH`. Earlier versions of picotool can't parse this firmware's
custom `binary_info` and fail with a cryptic `Hmm uncaught not mapped` error.
`flash-utility.ps1` checks the version itself and fails with a clear message
before touching the device.

```powershell
.\flash-utility.cmd <FIRMWARE> <BOARD_REV> <GEAR_RATIO>
```

Example:

```powershell
.\flash-utility.cmd build\commutator.elf J 2.0
```

`board_rev` is a single character. `gear_ratio` directly drives motor movement,
so the firmware enforces it must be greater than 0 and less than or equal to
100. An out-of-range value boots, but it reports the error over serial and
refuses to enable the motor.

After writing, the script reads the config sector directly back off the device
(via `picotool save`) and compares it byte-for-byte against what was intended,
while still in BOOTSEL mode. Success means that the correct bytes were written
to flash.
