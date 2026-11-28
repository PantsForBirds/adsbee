# Beast protocol

## Ports

* 30005: server -> client feed

## Format

All data is escaped: `0x1a` -> `0x1a 0x1a`. Note that synchronization is still
complex, since `0x1a 0x31` may be the start of a frame or mid-data, depending
on what preceded it. To synchronize, you must see, in order:
* != `0x1a`
* `0x1a`
* {`0x31`, `0x32`, `0x33`}

Escaping makes frame length for a given type variable, up to
`2 + (2 * data_length_sum)`


## Frame structure
* `0x1a`
* 1 byte frame type (see types below)
* 6 byte MLAT timestamp (see below)
  

## Frame types
* `0x31`: Mode-AC frame
  * 1 byte [RSSI](https://en.wikipedia.org/wiki/Received_signal_strength_indication)
  * 2 byte Mode-AC data
* `0x32`: Mode-S short frame
  * 1 byte RSSI
  * 7 byte Mode-S short data
* `0x33`: Mode-S long frame
  * 1 byte RSSI
  * 14 byte Mode-S long data
* `0x34`: Status data
  * *Appears to only be used by Mode-S Beast hardware later versions*
  * ?? byte status data
  * ?? byte DIP switch configuration


## MLAT timestamp
The MLAT timestamp included in each frame is the big-endian value of a 12 MHz
counter at the time of packet reception. This counter isn't calibrated to
external time, but receiving software can calculate its offset from other
receiving stations across multiple packets, and then use the differences between
station receive timing to calculate signal source position.

FlightAware's dump1090 fork sends `0x00 0x00 0x00 0x00 0x00 0x00` when it has
no MLAT data. ADSBee does the same for a packet without an MLAT timestamp
(`RawModeSPacket::kFlagNoMlatTimestamp`, set by the ADSBee m1421).


## RSSI
FlightAware's dump1090 fork sends `0xff` when it has no RSSI data.


## Examples

* `0x1a 0x32 0x08 0x3e 0x27 0xb6 0xcb 0x6a 0x1a 0x1a 0x00 0xa1 0x84 0x1a 0x1a 0xc3 0xb3 0x1d`
  * `0x1a`: Frame start
  * `0x32`: Mode-S short frame
  * `0x08 0x3e 0x27 0xb6 0xcb 0x6a`: MLAT counter value
    * Decimal: 9063047285610
  * `0x1a 0x1a`: Signal level
    * Unescaped: `0x1a`
    * Decimal: 26
    * 26 / 255 * 100% = 10%
  * `0x00 0xa1 0x84 0x1a 0x1a 0xc3 0xb3 0x1d`: Mode-S short data
    * Unescaped: `0x00 0xa1 0x84 0x1a 0xc3 0xb3 0x1d`


## Implementations

* [Mode-S Beast hardware](http://modesbeast.com/scope.html)
* [FlightAware dump1090 fork](https://flightaware.com/adsb/piaware/install)


## References

* [Original description](http://wiki.modesbeast.com/Mode-S_Beast:Data_Output_Formats)
