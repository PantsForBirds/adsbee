#!/bin/bash
# put.sh SERIAL IMAGE.uf2: IMAGE over found.bin, written over PICOBOOT (BOOTSEL via AT), wait for AT. Caller holds the lock.
cd ~/t0879 || exit 1
SER=$1; P=/dev/serial/by-id/usb-Pants_for_Birds_ADSBee_1090_$SER-if00
venv/bin/python uf2img.py ${FOUND:-found.bin} $2 $HOME/t0879/put.bin || exit 1
bash restore.sh $SER $HOME/t0879/put.bin | grep -E "differ|VERIFY|back"; rm -f $HOME/t0879/put.bin
for i in $(seq 40); do sleep 3; venv/bin/python usbat.py $P 3 "AT+DEVICE_INFO?" 2>/dev/null | grep -q "RP2040 Firmware Version" && break; done
sleep 15; venv/bin/python usbat.py $P 4 "AT+DEVICE_INFO?" "AT+CONFIG_HASH?" "AT+FEED_ENABLE?" 2>&1 | grep -a -E "Firmware Version|^FIRMWARE_RP2040|FEED_ENABLE="
