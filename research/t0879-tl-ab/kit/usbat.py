#!/usr/bin/env python3
"""Send AT commands to an ADSBee console; print responses. usage: usbat.py PORT WAIT CMD..."""
import serial, sys, time, re
s = serial.Serial(sys.argv[1], 115200, timeout=0.05)
wait = float(sys.argv[2]); s.read(1 << 20)
for c in sys.argv[3:]:
    s.write((c + "\r\n").encode()); end, out = time.time() + wait, b""
    while time.time() < end:
        out += s.read(65536)
        if re.search(rb"(^|\n)(OK|ERROR[^\r\n]*)\r?\n", out): break
    print(f">>> {c}\n" + re.sub(r"\x1b\[[0-9;]*m", "", out.decode(errors="replace")), flush=True)
