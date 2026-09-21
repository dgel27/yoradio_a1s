#!/usr/bin/env bash
# Flash prebuilt firmware + SPIFFS filesystem to ESP32-A1S via esptool.py.
# Run from anywhere; paths resolve relative to this script (yoRadio/).
#
# Usage: ./flash_all.sh [PORT] [BAUD]
#   PORT default: /dev/ttyUSB0, BAUD default: 460800
#
# Prerequisites (run once from yoRadio/):
#   pio run -e Yoradio_RELEASE_serialPort            # builds bootloader/partitions/firmware into _BUILD/
#   pio run -e Yoradio_RELEASE_serialPort -t buildfs # builds spiffs.bin from data/ into _BUILD/
#
# Flash offsets come from partition_1.5Mapp_OTA_0.9Mfs.csv:
#   bootloader @ 0x1000, partitions @ 0x8000, boot_app0 @ 0xe000,
#   firmware (app0) @ 0x10000, spiffs @ 0x310000.
set -euo pipefail

PORT="${1:-/dev/ttyUSB0}"
BAUD="${2:-460800}"
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD="$DIR/_BUILD/Yoradio_RELEASE_serialPort"

for f in bootloader.bin partitions.bin firmware.bin spiffs.bin; do
  if [ ! -f "$BUILD/$f" ]; then
    echo "Missing $BUILD/$f — run 'pio run' and 'pio run -t buildfs' first." >&2
    exit 1
  fi
done

BOOT_APP0="$(find "$HOME/.platformio/packages" -path "*framework-arduinoespressif32*/tools/partitions/boot_app0.bin" 2>/dev/null | head -1)"
if [ -z "$BOOT_APP0" ]; then
  echo "boot_app0.bin not found under ~/.platformio/packages" >&2
  exit 1
fi

esptool.py --chip esp32 --port "$PORT" --baud "$BAUD" \
  --before default_reset --after hard_reset write_flash -z \
  --flash_mode dio --flash_freq 40m --flash_size 4MB \
  0x1000 "$BUILD/bootloader.bin" \
  0x8000 "$BUILD/partitions.bin" \
  0xe000 "$BOOT_APP0" \
  0x10000 "$BUILD/firmware.bin" \
  0x310000 "$BUILD/spiffs.bin"
