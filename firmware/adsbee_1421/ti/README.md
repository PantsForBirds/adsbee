# ADSBee 1421 Firmware

## Flashing Instructions

You can flash the ADSBee m1421 using any flash utility that will connect to and program a TI CC1314R10 over JTAG. There are a number of options, many of which are inexpensive, but this guide contains instructions for the Segger JLink (same instructions apply for the JLink Base / Base Compact / Pro / etc).

1. Download the .hex firmware binary from the latest m1421 firmware release.
2. Set the target type to CC1314R10, and connect using JTAG / 4000kHz.
3. Select the .hex firmware binary and flash.