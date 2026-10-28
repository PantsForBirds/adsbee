#!/usr/bin/env python3
"""Restore an RP2040 flash dump over PICOBOOT (device in BOOTSEL): rewrite only differing 64 KB chunks,
verify by full read-back, then reboot. usage: picoboot_restore.py DUMP [ports e.g. 1.4]"""
import sys, struct, time, hashlib, usb.core, usb.util
img = open(sys.argv[1], "rb").read(); size = len(img)
ports = tuple(int(x) for x in (sys.argv[2] if len(sys.argv) > 2 else "1.4").split("."))
dev = next(d for d in usb.core.find(find_all=True, idVendor=0x2e8a, idProduct=0x0003) if d.port_numbers == ports)
intf = next(i for i in dev.get_active_configuration() if i.bInterfaceClass == 0xFF)
if dev.is_kernel_driver_active(intf.bInterfaceNumber): dev.detach_kernel_driver(intf.bInterfaceNumber)
usb.util.claim_interface(dev, intf.bInterfaceNumber)
ep_out = next(e for e in intf if usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT)
ep_in = next(e for e in intf if usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN)
dev.ctrl_transfer(0x41, 0x41, 0, intf.bInterfaceNumber, None)
tok = [1]
def cmd(cid, args=b"", xfer=0, data_in=False, data_out=None):
    hdr = struct.pack("<IIBBHI", 0x431fd10b, tok[0], cid, len(args), 0, xfer) + args.ljust(16, b"\0"); tok[0] += 1
    ep_out.write(hdr, timeout=5000)
    if data_in:
        data = bytes(ep_in.read(xfer, timeout=10000))
        while len(data) < xfer: data += bytes(ep_in.read(xfer - len(data), timeout=10000))
        ep_out.write(b"", timeout=5000); return data
    if data_out is not None:
        ep_out.write(data_out, timeout=20000)
    ep_in.read(1, timeout=20000)
CH = 64 << 10; BASE = 0x10000000
def read_all():
    return b"".join(cmd(0x84, struct.pack("<II", BASE + a, CH), CH, True) for a in range(0, size, CH))
cmd(0x01, struct.pack("<B", 1)); cmd(0x06)
cur = read_all()
diff = [a for a in range(0, size, CH) if cur[a:a + CH] != img[a:a + CH]]
print(f"{len(diff)} of {size // CH} 64 KB chunks differ: {[hex(BASE + a) for a in diff]}", flush=True)
t = time.time()
for a in diff:
    cmd(0x03, struct.pack("<II", BASE + a, CH))                      # FLASH_ERASE
    cmd(0x05, struct.pack("<II", BASE + a, CH), CH, data_out=img[a:a + CH])  # WRITE
print(f"rewrote in {time.time() - t:.1f} s", flush=True)
back = read_all()
ok = back == img
print("VERIFY", "OK" if ok else "MISMATCH", hashlib.sha256(back).hexdigest(), flush=True)
cmd(0x01, struct.pack("<B", 0))
if ok:
    try: cmd(0x02, struct.pack("<III", 0, 0, 500))
    except Exception as e: print("reboot:", e)
