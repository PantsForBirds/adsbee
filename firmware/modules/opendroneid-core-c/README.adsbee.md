# opendroneid-core-c (vendored subset)

Vendored subset of the [Open Drone ID Core C Library](https://github.com/opendroneid/opendroneid-core-c),
used by ADSBee 1090 to decode Broadcast Remote ID (ASTM F3411 / ASD-STAN prEN 4709-002) messages
received on the ESP32-S3 (BLE + WiFi) and forwarded to the RP2040.

- **Upstream commit:** `4b266c7c33e5299bfbe8427ed8518e869e3a7d7f` (2026-02-10)
- **License:** Apache-2.0 (see `LICENSE`)

## What is vendored

Only the portable, receive-side codec is vendored:

- `libopendroneid/opendroneid.c` / `opendroneid.h` — the message encoder/decoder. We use the
  decode path only: `decodeOpenDroneID()` (single 25-byte message) and `decodeMessagePack()`
  (up to 9 packed messages). Compiled with `-DODID_DISABLE_PRINTF` to drop the print helpers.
- `libopendroneid/odid_wifi.h` — IEEE 802.11 / NAN struct definitions used as a reference when
  parsing WiFi beacon Information Elements.

## What is intentionally omitted

- `libopendroneid/wifi.c` — pulls in `<Arduino.h>`, `<endian.h>`, `<byteswap.h>` (glibc/Arduino
  specific, not available under ESP-IDF newlib or the RP2040 SDK) and is transmit-focused. ADSBee
  performs its own transport framing (BLE Service Data AD structure, WiFi vendor-specific IE) in
  `firmware/adsbee_1090/esp/main/remote_id/` and calls the codec directly on the extracted
  ODID message / message-pack bytes.

## Updating

Re-copy the four files above from the upstream tag and re-run the host unit tests
(`firmware/common` googletest harness — Remote ID decode fixtures). Do not add `wifi.c` without
first porting away from the non-portable includes.
