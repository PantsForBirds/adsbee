#!/usr/bin/env python3
"""uf2img.py BASE.bin IMAGE.uf2 OUT.bin: flash dump BASE with every UF2 block of IMAGE written over it (RP2040 family)."""
import struct, sys
base = bytearray(open(sys.argv[1], 'rb').read()); u = open(sys.argv[2], 'rb').read(); n = 0
for o in range(0, len(u), 512):
    m0, m1, flags, addr, size, bno, nblk, fam = struct.unpack('<8I', u[o:o + 32])
    assert m0 == 0x0A324655 and m1 == 0x9E5D5157, 'not a UF2 block'
    if flags & 1: continue                       # not for the main flash
    assert 0x10000000 <= addr < 0x10000000 + len(base), hex(addr)
    a = addr - 0x10000000; base[a:a + size] = u[o + 32:o + 32 + size]; n += 1
open(sys.argv[3], 'wb').write(base); print(f'{n} blocks written over {sys.argv[1]}')
