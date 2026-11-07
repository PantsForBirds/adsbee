# Beast feeds

`AT+FEED=<index>,<uri>,<port>,<active>,<protocol>` picks what a feed sends:

| Protocol | Sends |
|---|---|
| `BEAST` | Mode S, UAT, UAT uplinks |
| `BEAST_NO_UAT_UPLINK` | Mode S, UAT |
| `BEAST_NO_UAT` | Mode S |

## Capabilities check

Every TCP Beast feed asks the server what else it accepts (the ADSBee Beast extensions (2026-11-06), which readsbee implements):

- Right after the feed UUID (`0xe4`), the ADSBee sends a 6-byte query (`0xe5 'Q'`).
- A readsbee server answers (`0xe5 'A'`). From the moment the ADSBee reads the answer it adds what both sides agreed
  on (the query bits AND the answer bits). Every frame says what it is, so no switch point is needed.
- Any other server (stock readsb, dump1090, aggregators) gets today's Beast stream plus those 6 bytes. Readsb counts
  them as 6 bytes of garbage, far below its 512-byte limit; nothing else changes.
- No answer within 5 s: plain Beast until the connection closes. Every new connection asks again.
- UDP, MQTT and serial Beast never ask and stay plain.

| Bit | Feature | Offered |
|---|---|---|
| 0 | Reserved: never set, ignored in an answer | never |
| 1 | ADS-L frames (`0xed`) | not yet |
| 2 | Remote ID frames (`0xef`) | while Remote ID receive is on |
| 3 | UAT timestamps on the Mode S clock (upper-case `S` / `L` / `U`) | not yet |


## Frames

Every frame starts with `0x1a` and a type byte. A `0x1a` after those is sent twice.

| Type | Frame | After the type byte | Sent |
|---|---|---|---|
| `0xe4` | Feed UUID, sent once on connect | 36 ASCII characters | TCP Beast feeds |
| `0xe5` | Capabilities | `Q` / `A` (`C` reserved), version (1), 2 bytes of feature bits | TCP Beast feeds |
| `0x32` / `0x33` | Mode S short / long | timestamp (6), signal (1), 7 / 14 bytes | all Beast |
| `0xec` | UAT | `s` / `l` / `u`, timestamp (6), signal (1), 30 / 48 / 552 bytes | not on `BEAST_NO_UAT` (`u` not on `BEAST_NO_UAT_UPLINK`) |
| `0xef` | Remote ID | link, timestamp (6), signal (1), MAC (6), ODID message or pack | bit 2 agreed |

Signal byte, on every frame type: the Beast power byte (0 = no level, 1 at -100 dBm and below, 255 at -40 dBm and
above).

Remote ID:

- Link letter: `b` Bluetooth 4 legacy, `c` Bluetooth 5 Long Range (Coded), `e` Bluetooth 5 extended 1M,
  `w` WiFi beacon, `n` WiFi NaN. The ADSBee receives `b`, `c` and `w`.
- Timestamp: once the ESP32 is synced to the MLAT clock, the packet's time on the Mode S 12 MHz clock with an
  upper-case link letter (ordering, not MLAT-grade); before that 0 with a lower-case letter, and the server stamps
  arrival time.
- MAC is the transmitter's address, most significant byte first.
- ODID: one 25-byte ASTM F3411 message, or a message pack (`0xF?`, size 25, count 1-9, then the messages).
- Every received packet is sent, including repeats. Packets received before the server's answer are not sent.
