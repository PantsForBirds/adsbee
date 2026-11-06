# Beast feeds

`AT+FEED=<index>,<uri>,<port>,<active>,<protocol>` picks what a feed sends:

| Protocol | Sends |
|---|---|
| `BEAST` | Mode S, UAT, UAT uplinks |
| `BEAST_NO_UAT_UPLINK` | Mode S, UAT |
| `BEAST_NO_UAT` | Mode S |
| `READSBEE` | everything `BEAST` sends, plus Remote ID |

Use `READSBEE` only for a readsbee server. Stock readsb and most aggregators don't know its extra frames and may
close the feed. `READSBEE` is for feeds only; a serial interface uses `BEAST`.

## Frames

Every frame starts with `0x1a` and a type byte. A `0x1a` after those is sent twice.

| Type | Frame | After the type byte | Feeds |
|---|---|---|---|
| `0xe4` | Feed UUID, sent once on connect | 36 ASCII characters | all Beast |
| `0x32` / `0x33` | Mode S short / long | timestamp (6), signal (1), 7 / 14 bytes | all Beast |
| `0xec` | UAT | `s` / `l` / `u`, timestamp (6), signal (1), 30 / 48 / 552 bytes | `BEAST`, `READSBEE` (`u` not on `BEAST_NO_UAT_UPLINK`) |
| `0xef` | Remote ID | link, timestamp (6), dBm (1), MAC (6), ODID message or pack | `READSBEE` |

Remote ID (readsbee feed protocol v2):

- Link letter: `b` Bluetooth 4 legacy, `c` Bluetooth 5 Long Range (Coded), `e` Bluetooth 5 extended 1M,
  `w` WiFi beacon, `n` WiFi NaN. The ADSBee receives `b`, `c` and `w`.
- Timestamp is 0: the ESP32 has no Mode S clock, so the server stamps arrival time.
- dBm is a signed byte; `0x80` (-128) means no level.
- MAC is the transmitter's address, most significant byte first.
- ODID: one 25-byte ASTM F3411 message, or a message pack (`0xF?`, size 25, count 1-9, then the messages).
- Every received packet is sent, including repeats.
