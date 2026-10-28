#!/bin/bash
# state.sh PORT TAG [extra AT...]: read-only state of a 1090; full text in priv/, redacted copy in runs/. Caller holds the lock.
cd ~/t0879 || exit 1
P=$1; T=$2; shift 2
venv/bin/python usbat.py $P 4 "AT+DEVICE_INFO?" "AT+UPTIME?" "AT+CONFIG_HASH?" "AT+FEED_ENABLE?" "AT+FEED?" "AT+REMOTE_ID?" "AT+REMOTE_ID_TX?" "AT+PROTOCOL_OUT?" "AT+LOG_LEVEL?" "AT+TL_OFFSET?" "AT+NETWORK_INFO?" "AT+ESP32_REBOOT_INFO?" "$@" > priv/$T-state.txt 2>&1
venv/bin/python usbat.py $P 6 "AT+SETTINGS?DUMP" > priv/$T-dump.txt 2>&1
red() { sed -E -e 's/((SSID|[Pp]ass(word)?|[Kk]ey|KEY|Token|TOKEN|[Ll]at(itude)?|[Ll]on(gitude)?|LAT|LON|UUID|URI|RECEIVER_ID|MAC|IP)[^:=,]*[:=]).*/\1 <redacted>/' -e 's/^(AT\+)?(WIFI_(AP|STA)|FEED|RX_POSITION|FEED_RECEIVER_ID|OTA_KEY|ESP32_OTA_KEY)=.*/\2=<redacted>/' ; }
red < priv/$T-state.txt > runs/$T-state.txt; red < priv/$T-dump.txt > runs/$T-dump.txt
grep -a -E "CONFIG_HASH|FIRMWARE|SETTINGS_FULL|UPTIME|FEED_ENABLE=|REMOTE_ID|PROTOCOL_OUT=|LOG_LEVEL=|TL_OFFSET=|Firmware|ERROR" runs/$T-state.txt | head -40
echo "dump lines: $(grep -c '^AT+' priv/$T-dump.txt)"
