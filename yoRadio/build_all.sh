#!/usr/bin/env bash
# Build yoRadio firmware + SPIFFS filesystem for ESP32-A1S via PlatformIO.
# Run from anywhere; paths resolve relative to this script (yoRadio/).
#
# Usage: ./build_all.sh
#
# Builds (artifacts land in _BUILD/Yoradio_RELEASE_serialPort/):
#   firmware.bin  - bootloader, partitions, firmware
#   spiffs.bin    - SPIFFS image compiled from data/
#
# Requirements: PlatformIO CLI (`pio`) in PATH.
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD="$DIR/_BUILD/Yoradio_RELEASE_serialPort"

if ! command -v pio >/dev/null 2>&1; then
  echo "pio not found in PATH - install PlatformIO CLI first." >&2
  exit 1
fi

echo "=== Building firmware (Yoradio_RELEASE_serialPort) ==="
(cd "$DIR" && pio run -e Yoradio_RELEASE_serialPort)

echo "=== Building SPIFFS filesystem (data/ -> spiffs.bin) ==="
(cd "$DIR" && pio run -e Yoradio_RELEASE_serialPort -t buildfs)

FW="$BUILD/firmware.bin"
FS="$BUILD/spiffs.bin"
for f in "$FW" "$FS"; do
  if [ ! -f "$f" ]; then
    echo "Build succeeded but $f is missing - check PlatformIO output above." >&2
    exit 1
  fi
done

echo
echo "Build complete:"
echo "  $FW  ($(stat -c %s "$FW") bytes)"
echo "  $FS  ($(stat -c %s "$FS") bytes)"