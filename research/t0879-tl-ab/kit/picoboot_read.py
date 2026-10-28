#!/usr/bin/env python3
"""Read RP2040 flash over PICOBOOT (device in BOOTSEL on USB port path 1-<ports>). usage: picoboot_read.py OUT SIZE_MB [ports e.g. 1.4]"""
import sys, struct, time, usb.core, usb.util
out, size = sys.argv[1], int(sys.argv[2]) << 20
ports = tuple(int(x) for x in (sys.argv[3] if len(sys.argv) > 3 else "1.4").split("."))
dev = next(d for d in usb.core.find(find_all=True, idVendor=0x2e8a, idProduct=0x0003) if d.port_numbers == ports)
cfg = dev.get_active_configuration()
intf = next(i for i in cfg if i.bInterfaceClass == 0xFF)
if dev.is_kernel_driver_active(intf.bInterfaceNumber): dev.detach_kernel_driver(intf.bInterfaceNumber)
usb.util.claim_interface(dev, intf.bInterfaceNumber)
ep_out = next(e for e in intf if usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT)
ep_in = next(e for e in intf if usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN)
dev.ctrl_transfer(0x41, 0x41, 0, intf.bInterfaceNumber, None)  # PICOBOOT interface reset
tok = [1]
def cmd(cid, args=b"", xfer=0, data_in=False):
    hdr = struct.pack("<IIBBHI", 0x431fd10b, tok[0], cid, len(args), 0, xfer) + args.ljust(16, b"\0"); tok[0] += 1
    ep_out.write(hdr, timeout=5000)
    data = None
    if data_in:
        data = bytes(ep_in.read(xfer, timeout=10000))
        while len(data) < xfer: data += bytes(ep_in.read(xfer - len(data), timeout=10000))
        ep_out.write(b"", timeout=5000)
    else:
        ep_in.read(1, timeout=5000)  # zero-length ack
    return data
cmd(0x01, struct.pack("<B", 1))     # EXCLUSIVE_ACCESS
cmd(0x06)                            # EXIT_XIP
t = time.time(); CH = 64 << 10; buf = bytearray()
for addr in range(0x10000000, 0x10000000 + size, CH):
    buf += cmd(0x84, struct.pack("<II", addr, CH), CH, True)
open(out, "wb").write(buf)
print(f"read {len(buf)} B in {time.time()-t:.1f} s")
cmd(0x01, struct.pack("<B", 0))     # release exclusive access
if "--reboot" in sys.argv:
    try: cmd(0x02, struct.pack("<III", 0, 0, 500))  # REBOOT into the application after 500 ms
    except Exception as e: print("reboot sent", e)
usb.util.release_interface(dev, intf.bInterfaceNumber)
