#!/bin/bash
# run79.sh START_EPOCH FROM TO: wait until START_EPOCH, then until the 1090U is free and shows the lab image and
# settings (CONFIG_HASH 0x02FEF0CF, SETTINGS_FULL 0x4DB591F5), then run blocks FROM..TO-1. Checks every 5 min.
cd ~/t0879 || exit 1
T=$1; FROM=$2; TO=$3; SER=E46478B14B4C1F23; P=/dev/serial/by-id/usb-Pants_for_Birds_ADSBee_1090_$SER-if00
WANT_CH=0x02FEF0CF; WANT_SF=0x4DB591F5
while [ $(date +%s) -lt $T ]; do sleep 30; done
while [ ! -e STOP ]; do
  if pgrep -f "bash ab79.sh" > /dev/null; then sleep 300; continue; fi
  ok=$( exec 8>/tmp/adsbee-hil-locks/$SER.lock; flock -n 8 || { echo busy; exit; }
        venv/bin/python usbat.py $P 3 "AT+CONFIG_HASH?" "AT+TL_OFFSET?" 2>&1 | tr -d '\r' > runs/check-$FROM.txt
        grep -q "CONFIG_HASH=$WANT_CH" runs/check-$FROM.txt && grep -q "SETTINGS_FULL=$WANT_SF" runs/check-$FROM.txt && echo ok || echo mismatch )
  echo "$(date -u +%FT%T) run79 check before index $FROM: $ok $(grep -h -E 'CONFIG_HASH=|SETTINGS_FULL=' runs/check-$FROM.txt 2>/dev/null | tr '\n' ' ')" >> runs/ab.log
  [ "$ok" = ok ] && exec bash ab79.sh 1800 $FROM $TO
  sleep 300
done
