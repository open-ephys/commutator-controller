<#
.SYNOPSIS
    Flash the commutator firmware together with its persistent hardware
    configuration.

.DESCRIPTION
    The board revision and gear ratio are written to the last sector of flash,
    which the firmware image never reaches, so they survive future firmware
    flashes (see eeprom.cpp). A firmware built against boards/commutator.h
    can't overlap that sector at all -- the linker fails outright ("region
    'FLASH' overflowed") for any build that would.

    Config is always written after firmware, then read directly back off the
    device (via `picotool save`) and compared byte-for-byte against what was
    intended -- so success means "verified on flash", not just "picotool
    returned 0". This happens while still in BOOTSEL mode, before the final
    reboot, so it doesn't depend on the board enumerating over serial, and it
    catches a bad write regardless of what wrote it.

    Requires picotool (https://github.com/raspberrypi/picotool) 2.1.0 or later
    on PATH. Earlier versions of picotool can't parse this firmware's custom
    binary_info and fails with a cryptic "Hmm uncaught not mapped" error.

.EXAMPLE
    .\flash-utility.ps1 build\commutator.elf J 2.0
#>

param(
    [Parameter(Mandatory = $true, Position = 0)]
    [string]$Firmware,

    [Parameter(Mandatory = $true, Position = 1)]
    [string]$BoardRev,

    [Parameter(Mandatory = $true, Position = 2)]
    [string]$GearRatio
)

Set-StrictMode -Version Latest

# gear_ratio field width; must not exceed EEPROM_GEAR_RATIO_MAX_LEN in
# eeprom.cpp.
$GearField = 21

# picotool 2.0.0 can't parse the custom binary_info entry eeprom.cpp adds
# (eeprom_config_addr) and fails with an unhelpful internal error instead of a
# clean one, so it's worth checking for up front.
$MinPicotoolVersion = [version]"2.1.0"

function Die {
    param([string]$Message)
    Write-Host "error: $Message" -ForegroundColor Red
    exit 1
}

# --- Validate inputs -------------------------------------------------------

if (-not (Get-Command picotool -ErrorAction SilentlyContinue)) {
    Die "picotool not found on PATH"
}

$verOutput = & picotool version -s 2>&1
if ($LASTEXITCODE -ne 0) {
    Die "could not determine picotool version: $verOutput"
}
try {
    $picotoolVersion = [version]($verOutput -replace '-.*$', '')
} catch {
    Die "could not parse picotool version '$verOutput'"
}
if ($picotoolVersion -lt $MinPicotoolVersion) {
    Die ("picotool $verOutput is too old (need $MinPicotoolVersion or later) -- " +
         "it can't read this firmware's custom binary_info. Upgrade picotool.")
}

if (-not (Test-Path -LiteralPath $Firmware -PathType Leaf)) {
    Die "firmware file not found: $Firmware"
}

# Trim incidental whitespace
$BoardRev = $BoardRev.Trim()
$GearRatio = $GearRatio.Trim()

# check that board revision fits the 1-byte field, so the [char] cast below
# doesn't throw.
if ($BoardRev.Length -ne 1) {
    Die "board revision must be exactly one character"
}

# The gear ratio is stored verbatim as a decimal string; the firmware parses
# it with strtod at boot and rejects an out-of-range value. It must fit its
# NUL-terminated field, leaving room for the terminator.
if ($GearRatio.Length -ge $GearField) {
    Die "gear ratio '$GearRatio' too long (max $($GearField - 1) characters)"
}

# Reject anything that isn't a plain non-negative decimal (no letters, signs,
# exponents, or extra dots) up front, rather than relying solely on the
# firmware's strtod check at boot.
if ($GearRatio -notmatch '^[0-9]+(\.[0-9]+)?$') {
    Die "gear ratio '$GearRatio' must be a plain decimal number, e.g. 2.0"
}

# Read the config address out of the firmware's own binary_info
# (eeprom_config_addr, set in eeprom.cpp) instead of recomputing it here. This
# guarantees we write to wherever this exact firmware reads from.
$configOutput = & picotool config $Firmware 2>&1
if ($LASTEXITCODE -ne 0) {
    Die "picotool could not read firmware config: $configOutput"
}
$configAddrLine = $configOutput | Select-String -Pattern 'eeprom_config_addr\s*=\s*(\d+)'
if (-not $configAddrLine) {
    Die ("firmware does not expose eeprom_config_addr (via picotool config) -- " +
         "rebuild with the current eeprom.cpp")
}
$ConfigAddrValue = [int64]$configAddrLine.Matches[0].Groups[1].Value
$ConfigAddr = "0x{0:X8}" -f $ConfigAddrValue

# --- Build the config record -------------------------------------------------
# Layout must match struct eeprom_config_t in eeprom.cpp:
#   uint32 magic 0xDEADBEEF (LE) | char board_rev | char gear_ratio[N]

$magicBytes = [byte[]](0xEF, 0xBE, 0xAD, 0xDE)          # 0xDEADBEEF, little-endian
try {
    $revByte = [byte][char]$BoardRev                    # board_rev (1 byte)
} catch {
    Die "board revision must fit in a single byte (got '$BoardRev')"
}

$gearBytes  = [System.Text.Encoding]::ASCII.GetBytes($GearRatio)
$gearField  = New-Object byte[] $GearField              # zero-initialized -> NUL padding
[Array]::Copy($gearBytes, $gearField, $gearBytes.Length)

$configBytes = $magicBytes + $revByte + $gearField

$configFile   = Join-Path $env:TEMP ("flash-utility-config-{0}.bin" -f ([guid]::NewGuid()))
$readbackFile = Join-Path $env:TEMP ("flash-utility-readback-{0}.bin" -f ([guid]::NewGuid()))

try {
    [System.IO.File]::WriteAllBytes($configFile, $configBytes)

    Write-Host "Booting into bootloader mode"
    picotool reboot -f -u
    if ($LASTEXITCODE -ne 0) { Die "picotool reboot failed" }

    # Give the board time to re-enumerate as a USB BOOTSEL device before
    # trying to talk to it. picotool load can race the reboot otherwise.
    Start-Sleep -Seconds 2

    Write-Host "Flashing firmware: $Firmware"
    picotool load $Firmware
    if ($LASTEXITCODE -ne 0) { Die "picotool load (firmware) failed" }

    Write-Host "Writing config: board_rev='$BoardRev', gear_ratio=$GearRatio"
    picotool load $configFile -o $ConfigAddr
    if ($LASTEXITCODE -ne 0) { Die "picotool load (config) failed" }

    $configAddrEnd = "0x{0:X8}" -f ($ConfigAddrValue + $configBytes.Length)
    $saveOutput = & picotool save -r $ConfigAddr $configAddrEnd $readbackFile 2>&1
    if ($LASTEXITCODE -ne 0) { Die "picotool save (readback) failed: $saveOutput" }

    $readbackBytes = [System.IO.File]::ReadAllBytes($readbackFile)
    if (Compare-Object $configBytes $readbackBytes -SyncWindow 0) {
        Die "config readback does not match what was written -- flash may be corrupt"
    }
    Write-Host "Verified on flash: board_rev='$BoardRev', gear_ratio=$GearRatio"

    picotool reboot
    if ($LASTEXITCODE -ne 0) { Die "picotool reboot failed" }

    Write-Host "Done."
}
finally {
    Remove-Item -LiteralPath $configFile -ErrorAction SilentlyContinue
    Remove-Item -LiteralPath $readbackFile -ErrorAction SilentlyContinue
}
