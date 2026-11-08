#!/usr/bin/env bash
set -euo pipefail

# Flash a UF2. Prefer the RPI-RP2 mass-storage mount (hold BOOTSEL while
# plugging in); mount it with udisksctl when needed; fall back to picotool.

DIR=$(dirname -- "${BASH_SOURCE[0]}")
if [ $# -ge 1 ]; then
    UF2="$1"
else
    UF2="$DIR/main.uf2"
fi

if [ ! -f "$UF2" ]; then
    echo "missing $UF2; run ./build.sh first" >&2
    exit 1
fi

find_mount() {
    find /run/media /media /mnt -maxdepth 3 -type d -name 'RPI-RP2' \
        2>/dev/null | head -n1
}

MOUNT=$(find_mount || true)
if [ -z "$MOUNT" ] && command -v lsblk >/dev/null 2>&1 &&
    command -v udisksctl >/dev/null 2>&1; then
    DEV=$(lsblk -rno NAME,LABEL,MOUNTPOINT 2>/dev/null |
          awk '$2=="RPI-RP2" && $3==""{print "/dev/"$1; exit}')
    if [ -n "$DEV" ]; then
        udisksctl mount -b "$DEV" >/dev/null 2>&1 || true
        MOUNT=$(find_mount || true)
    fi
fi

if [ -n "$MOUNT" ]; then
    echo "flashing to $MOUNT"
    cp "$UF2" "$MOUNT/"
    sync
    echo "flashed $UF2"
    exit 0
fi

# A Pico W that is running the application is not a mass-storage device.
APP_PID=""
for d in /sys/bus/usb/devices/*; do
    if [ -f "$d/idVendor" ] &&
        [ "$(cat "$d/idVendor" 2>/dev/null)" = "2e8a" ]; then
        pid=$(cat "$d/idProduct" 2>/dev/null || true)
        if [ "$pid" != "0003" ]; then
            APP_PID="$pid"
        fi
    fi
done
if [ -n "$APP_PID" ]; then
    echo "the Pico W is running firmware (USB PID $APP_PID), not BOOTSEL" >&2
    echo "hold BOOTSEL and replug to get the RPI-RP2 mount" >&2
    exit 1
fi

if ! command -v picotool >/dev/null 2>&1; then
    echo "no RPI-RP2 mount and picotool not found" >&2
    echo "hold BOOTSEL and replug the Pico W, or run inside 'nix develop'" >&2
    exit 1
fi

INFO=$(picotool info 2>&1 || true)
if printf '%s\n' "$INFO" | grep -q 'unable to connect'; then
    echo "the RP2040 is in BOOTSEL but this user cannot access it" >&2
    echo "mount the RPI-RP2 volume, or run:" >&2
    echo "  sudo env \"PATH=\$PATH\" ./flash.sh $*" >&2
    exit 1
fi

if ! printf '%s\n' "$INFO" | grep -qE 'RP-series|RP2040'; then
    echo "no RP2040 in BOOTSEL mode; trying a USB reboot..." >&2
    picotool reboot -u >/dev/null 2>&1 || true
    sleep 2
    INFO=$(picotool info 2>&1 || true)
    if printf '%s\n' "$INFO" | grep -q 'unable to connect'; then
        echo "the RP2040 is in BOOTSEL but this user cannot access it" >&2
        echo "mount the RPI-RP2 volume, or run:" >&2
        echo "  sudo env \"PATH=\$PATH\" ./flash.sh $*" >&2
        exit 1
    fi
    if ! printf '%s\n' "$INFO" | grep -qE 'RP-series|RP2040'; then
        echo "still no BOOTSEL device; hold BOOTSEL and replug the Pico W" >&2
        exit 1
    fi
fi

picotool load -x "$UF2"
echo "flashed $UF2"
