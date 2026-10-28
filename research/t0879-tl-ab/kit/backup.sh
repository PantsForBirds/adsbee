#!/bin/bash
# backup.sh SERIAL OUT: reboot a 1090 into BOOTSEL, read 16 MB over PICOBOOT, reboot to the app. Caller holds the lock.
cd ~/t0879 || exit 1
SER=$1; OUT=$2
dev=$(grep -l "^$SER$" /sys/bus/usb/devices/*/serial | head -1); [ -n "$dev" ] && port=$(basename $(dirname $dev)) || port=1-1.2.2; ports=${port#1-}
echo "serial $SER at usb $port"
venv/bin/python usbat.py /dev/serial/by-id/usb-Pants_for_Birds_ADSBee_1090_$SER-if00 2 "AT+BOOT_USB_UF2=1DEADBEE" | tail -2
for i in $(seq 30); do sleep 1; [ "$(cat /sys/bus/usb/devices/$port/idProduct 2>/dev/null)" = "0003" ] && break; done
echo "idProduct $(cat /sys/bus/usb/devices/$port/idProduct)"
sleep 1
venv/bin/python picoboot_read.py $OUT 16 $ports --reboot && sha256sum $OUT
for i in $(seq 40); do sleep 1; [ -e /dev/serial/by-id/usb-Pants_for_Birds_ADSBee_1090_$SER-if00 ] && break; done
sleep 3; echo "app back after ~$i s"
