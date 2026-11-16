#!/usr/bin/env python3
"""Image tools for the SoC.

    mkimage.py hex   in.bin out.hex [--words N]     $readmemh image, one 32-bit word per line
    mkimage.py image in.bin out.img [--load A | --xip] [--entry A]
                                                    boot image for SPI flash (iceprog -o 1M)

A boot image is a 20-byte header (magic, load address, size, entry, CRC-32)
followed by the program. With --xip the program is executed in place from
flash; otherwise the boot ROM copies it to the load address.
"""
import argparse
import struct
import sys
import zlib

BOOT_MAGIC = 0x42355652          # "RV5B"
BOOT_XIP = 0xFFFFFFFF


def to_hex(data, words=None):
    data += b"\0" * (-len(data) % 4)
    values = [w[0] for w in struct.iter_unpack("<I", data)]
    if words is not None:
        if len(values) > words:
            sys.exit(f"image is {len(values)} words, only {words} fit")
        values += [0] * (words - len(values))
    return "".join(f"{v:08x}\n" for v in values)


def boot_image(payload, load=None, entry=None):
    payload += b"\0" * (-len(payload) % 4)
    load_addr = BOOT_XIP if load is None else load
    if entry is None:
        entry = 0 if load is None else load
    header = struct.pack("<5I", BOOT_MAGIC, load_addr, len(payload), entry,
                         zlib.crc32(payload) & 0xFFFFFFFF)
    return header + payload


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    h = sub.add_parser("hex")
    h.add_argument("src")
    h.add_argument("dst")
    h.add_argument("--words", type=int)
    i = sub.add_parser("image")
    i.add_argument("src")
    i.add_argument("dst")
    i.add_argument("--load", type=lambda s: int(s, 0), default=0)
    i.add_argument("--xip", action="store_true")
    i.add_argument("--entry", type=lambda s: int(s, 0))
    args = ap.parse_args()

    data = open(args.src, "rb").read()
    if args.cmd == "hex":
        open(args.dst, "w", newline="\n").write(to_hex(data, args.words))
    else:
        open(args.dst, "wb").write(boot_image(data, None if args.xip else args.load, args.entry))
    print(f"{args.dst}: {len(data)} bytes")


if __name__ == "__main__":
    main()
