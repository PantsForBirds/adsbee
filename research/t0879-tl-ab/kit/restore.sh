#!/bin/bash
# restore.sh SERIAL DUMP: BOOTSEL via AT, PICOBOOT rewrite of differing 64 KB chunks + full verify + reboot. Caller holds the lock.
cd ~/t0879 || exit 1
SER=$1; DUMP=$2
dev=$(grep -l "^$SER$" /sys/bus/usb/devices/*/serial | head -1); [ -n "$dev" ] && port=$(basename $(dirname $dev)) || port=1-1.2.2; ports=${port#1-}
venv/bin/python usbat.py /dev/serial/by-id/usb-Pants_for_Birds_ADSBee_1090_$SER-if00 2 "AT+BOOT_USB_UF2=1DEADBEE" >/dev/null 2>&1
for i in $(seq 30); do sleep 1; [ "$(cat /sys/bus/usb/devices/$port/idProduct 2>/dev/null)" = "0003" ] && break; done
sleep 1; echo "BOOTSEL at $port after ${i}s"
venv/bin/python picoboot_restore.py $DUMP $ports
for i in $(seq 40); do sleep 1; [ -e /dev/serial/by-id/usb-Pants_for_Birds_ADSBee_1090_$SER-if00 ] && break; done; echo "app back after ~${i}s"
