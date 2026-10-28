#!/bin/bash
# block79.sh TL TAG SECS: set the TL offset (runtime only, not saved), log the console for SECS with LAB_RX interval
# lines (#LS, 10 s; #LP per decoded packet) and a comparator probe every 15 s (AT+LABDUTY=50, AT+LABRAW=512). TL=LEARN runs
# AT+TL_OFFSET=LEARN first (about 20 min) and logs it. Holds the unit's lock.
cd ~/t0879 || exit 1
TL=$1; TAG=$2; SECS=$3; SER=E46478B14B4C1F23; P=/dev/serial/by-id/usb-Pants_for_Birds_ADSBee_1090_$SER-if00
exec 8>/tmp/adsbee-hil-locks/$SER.lock; flock -w 3600 8 || { echo "$(date -u +%FT%T) LOCK TIMEOUT"; exit 1; }
echo "worker-5 t-0879 TL A/B block $TAG" >&8
echo "$(date -u +%FT%T) block $TAG start"
if [ "$TL" = LEARN ]; then
  venv/bin/python con79.py $P 1500 runs/$TAG-learn.log --at "AT+LABRX=0,0,10000" "AT+TL_OFFSET=LEARN" "AT+TL_OFFSET?" --end "AT+TL_OFFSET?"
  SET="AT+TL_OFFSET?"
else
  SET="AT+TL_OFFSET=$TL"
fi
echo "$(date -u +%FT%T) block $TAG logging"
venv/bin/python con79.py $P $SECS runs/$TAG.log --at "$SET" "AT+TL_OFFSET?" "AT+UPTIME?" "AT+CONFIG_HASH?" "AT+LABRX=1,0,10000,0" "AT+LABTRIG?" \
  --every 15 "AT+LABDUTY=50" "AT+LABRAW=512" --end "AT+LABRX=0" "AT+TL_OFFSET?" "AT+TL_READ?" "AT+UPTIME?"
echo "$(date -u +%FT%T) block $TAG done"
