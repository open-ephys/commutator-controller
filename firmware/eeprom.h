#pragma once

// Read-only access to a persistent configuration region stored in the last
// sector of flash. Firmware images are written from the start of flash and
// never reach this region, so values written here (e.g. with picotool during
// production) survive firmware flashes. See the README.md in this directory for
// the provisioning workflow.

// Reads the stored board revision character into `*out`. Returns true if the
// config region is provisioned; the caller validates the character itself.
// `*out` is left unchanged otherwise.
bool eeprom_read_board_rev(char *out);

// Reads the stored gear ratio into `*out`. Returns true if a valid config
// region was found; `*out` is left unchanged otherwise.
bool eeprom_read_gear_ratio(double *out);
