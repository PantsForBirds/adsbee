#!/usr/bin/env python3
"""con79.py PORT SECONDS OUT [--at CMD]... [--end CMD]... [--every N CMD...]: send --at commands, log console lines
(unix time prefix) for SECONDS, sending the --every commands every N seconds, then send --end commands and keep logging
3 s. One serial session for the whole run."""
import serial, sys, time, re
port, secs, out = sys.argv[1], float(sys.argv[2]), sys.argv[3]
at, end, every, period, mode = [], [], [], 0, None
for a in sys.argv[4:]:
    if a in ('--at', '--end', '--every'): mode = a; continue
    if mode == '--every' and not period: period = float(a); continue
    {'--at': at, '--end': end, '--every': every}[mode].append(a)
s = serial.Serial(port, 115200, timeout=0.2)
f = open(out, 'a', buffering=1); buf = b''
def pump(until):
    global buf
    while time.time() < until:
        try: d = s.read(65536)
        except serial.SerialException as e: f.write(f'{time.time():.3f} !SERIAL {e}\n'); time.sleep(1); continue
        if not d: continue
        buf += d; now = time.time()
        while b'\n' in buf:
            l, buf = buf.split(b'\n', 1)
            f.write(f'{now:.3f} ' + re.sub(r'\x1b\[[0-9;]*m', '', l.decode(errors='replace')).rstrip('\r') + '\n')
def send(c):
    f.write(f'{time.time():.3f} >>> {c}\n'); s.write((c + '\r\n').encode()); s.flush(); pump(time.time() + 1.5)
for c in at: send(c)
t_end = time.time() + secs
while time.time() < t_end:
    if every:
        for c in every: send(c)
        pump(min(t_end, time.time() + period))
    else:
        pump(t_end)
for c in end: send(c)
pump(time.time() + 3)
