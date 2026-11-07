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
- A readsbee server answers (`0xe5 'A'`). The ADSBee then sends a confirm (`0xe5 'C'`) and, from there on, only what
  both sides agreed on.
- Any other server (stock readsb, dump1090, aggregators) gets today's Beast stream plus those 6 bytes. Readsb counts
  them as 6 bytes of garbage, far below its 512-byte limit; nothing else changes.
- No answer within 5 s: plain Beast until the connection closes. Every new connection asks again.
- UDP, MQTT and serial Beast never ask and stay plain.

| Bit | Feature | Offered |
|---|---|---|
| 0 | Signal levels as signed dBm instead of the Beast power byte | always |
| 1 | ADS-L frames (`0xed`) | not yet |
| 2 | Remote ID frames (`0xef`) | while Remote ID receive is on |
| 3 | UAT timestamps on the Mode S clock (upper-case `S` / `L` / `U`) | not yet |

Signal levels are an uncalibrated estimate with 1 dB resolution, in dBm or as a power byte.

## Frames

Every frame starts with `0x1a` and a type byte. A `0x1a` after those is sent twice.

| Type | Frame | After the type byte | Sent |
|---|---|---|---|
| `0xe4` | Feed UUID, sent once on connect | 36 ASCII characters | TCP Beast feeds |
| `0xe5` | Capabilities | `Q` / `A` / `C`, version (1), 2 bytes of feature bits | TCP Beast feeds |
| `0x32` / `0x33` | Mode S short / long | timestamp (6), signal (1), 7 / 14 bytes | all Beast |
| `0xec` | UAT | `s` / `l` / `u`, timestamp (6), signal (1), 30 / 48 / 552 bytes | not on `BEAST_NO_UAT` (`u` not on `BEAST_NO_UAT_UPLINK`) |
| `0xef` | Remote ID | link, timestamp (6), signal (1), MAC (6), ODID message or pack | bit 2 agreed |

Signal byte: the Beast power byte (0 = no level, 1 at -100 dBm and below, 255 at -40 dBm and above), or with bit 0
agreed a signed dBm byte (`0x80` = no level).

Remote ID:

- Link letter: `b` Bluetooth 4 legacy, `c` Bluetooth 5 Long Range (Coded), `e` Bluetooth 5 extended 1M,
  `w` WiFi beacon, `n` WiFi NaN. The ADSBee receives `b`, `c` and `w`.
- Timestamp is 0: the ESP32 has no Mode S clock, so the server stamps arrival time.
- MAC is the transmitter's address, most significant byte first.
- ODID: one 25-byte ASTM F3411 message, or a message pack (`0xF?`, size 25, count 1-9, then the messages).
- Every received packet is sent, including repeats. Packets received before the server's answer are not sent.
