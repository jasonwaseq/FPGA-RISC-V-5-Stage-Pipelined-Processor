#!/usr/bin/env python3
"""Host side of the boot ROM loader protocol.

Used by the hardware test suite, the command-line tool and (through a
simulated serial port) the full-system simulation, so the same code talks to
the simulated SoC and to the board.

    board = Board("/dev/ttyUSB1")
    board.reset()                        # UART break, then claim the loader
    board.load(0, open("hello.bin", "rb").read())
    result = board.run(0)                # Exit(code, output) or Trap(...)
"""
import struct
import sys
import time
import zlib
from dataclasses import dataclass

RAM_TOP = 0x0002_0000
TRAP_FRAME = RAM_TOP - 128
ESC = 0x1B

CAUSES = {
    0: "instruction address misaligned", 1: "instruction access fault",
    2: "illegal instruction", 3: "breakpoint", 4: "load address misaligned",
    5: "load access fault", 6: "store address misaligned", 7: "store access fault",
    11: "environment call",
}


@dataclass
class Exit:
    code: int
    output: bytes


@dataclass
class Trap:
    cause: int
    epc: int
    tval: int
    output: bytes

    def __str__(self):
        name = CAUSES.get(self.cause, f"interrupt {self.cause & 0xFF}" if self.cause >> 31 else "?")
        return f"trap: {name} (mcause={self.cause:#x}) at pc={self.epc:#010x}, mtval={self.tval:#010x}"


class ProtocolError(RuntimeError):
    pass


class Board:
    CHUNK = 256             # bytes per write command; the receive FIFO holds 512

    def __init__(self, port=None, baud=115200, serial_port=None, timeout=2.0):
        if serial_port is None:
            import serial
            serial_port = serial.Serial(port, baud, timeout=timeout)
        self.ser = serial_port

    def close(self):
        self.ser.close()

    # ---- raw transport -------------------------------------------------------
    def _read(self, n, what="reply"):
        data = self.ser.read(n)
        if len(data) != n:
            raise ProtocolError(f"timeout waiting for {what} ({len(data)}/{n} bytes)")
        return data

    def _expect_ok(self, what):
        ack = self._read(1, what)
        if ack != b"K":
            raise ProtocolError(f"{what}: unexpected reply {ack!r}")

    # ---- taking control ------------------------------------------------------
    def reset(self, attempts=40):
        """Reset the SoC with a UART break and claim the boot ROM loader."""
        self.ser.send_break(0.05)
        self.sync(attempts)

    def sync(self, attempts=40):
        """Send '!' until the loader answers; the ROM banner is skipped."""
        old = self.ser.timeout
        self.ser.timeout = 0.05
        try:
            self.ser.reset_input_buffer()
            for _ in range(attempts):
                self.ser.write(b"!")
                data = self.ser.read(64)
                if data.endswith(b"K"):
                    # Further '!' may be queued: drain their replies.
                    while self.ser.read(64):
                        pass
                    return
            raise ProtocolError("the boot ROM loader did not answer")
        finally:
            self.ser.timeout = old

    # ---- memory --------------------------------------------------------------
    def write(self, addr, data):
        for off in range(0, len(data), self.CHUNK):
            chunk = data[off:off + self.CHUNK]
            self.ser.write(b"W" + struct.pack("<II", addr + off, len(chunk)) + chunk)
            self._expect_ok("write")

    def read(self, addr, n):
        self.ser.write(b"R" + struct.pack("<II", addr, n))
        return self._read(n, "memory contents")

    def read_words(self, addr, count):
        return list(struct.unpack(f"<{count}I", self.read(addr, 4 * count)))

    def write_words(self, addr, words):
        self.write(addr, struct.pack(f"<{len(words)}I", *[w & 0xFFFFFFFF for w in words]))

    def crc(self, addr, n):
        self.ser.write(b"C" + struct.pack("<II", addr, n))
        old = self.ser.timeout
        self.ser.timeout = max(old or 0, 2.0 + n / 20000)
        try:
            return struct.unpack("<I", self._read(4, "CRC"))[0]
        finally:
            self.ser.timeout = old

    def load(self, addr, data):
        """Write and verify with the ROM's CRC of what actually landed in memory."""
        self.write(addr, data)
        got, want = self.crc(addr, len(data)), zlib.crc32(data) & 0xFFFFFFFF
        if got != want:
            raise ProtocolError(f"CRC mismatch after load: {got:#010x} != {want:#010x}")

    # ---- execution -----------------------------------------------------------
    def go(self, addr):
        self.ser.write(b"G" + struct.pack("<I", addr))
        self._expect_ok("go")

    def wait_report(self, timeout=10.0, echo=None, feed=None):
        """Collect program output until the ROM reports an exit or a trap.

        `feed`, if given, is called with the output so far and may return bytes
        to send to the program (for interactive tests).
        """
        out = bytearray()
        deadline = time.time() + timeout
        old = self.ser.timeout
        self.ser.timeout = 0.05
        try:
            while True:
                data = self.ser.read(256)
                if data:
                    out += data
                    if echo:
                        echo(data)
                    if feed:
                        reply = feed(bytes(out))
                        if reply:
                            self.ser.write(reply)
                report = self._parse_report(out)
                if report:
                    return report
                if time.time() > deadline:
                    raise ProtocolError(f"program did not finish; output so far: {bytes(out)!r}")
        finally:
            self.ser.timeout = old

    def _parse_report(self, out):
        i = out.rfind(bytes([ESC]))
        if i < 0 or i + 1 >= len(out):
            return None
        kind, body = out[i + 1], out[i + 2:]
        if kind == ord("X") and len(body) >= 4:
            return Exit(struct.unpack("<I", body[:4])[0], bytes(out[:i]))
        if kind == ord("T") and len(body) >= 12:
            return Trap(*struct.unpack("<III", body[:12]), bytes(out[:i]))
        return None

    def run(self, addr, timeout=10.0, echo=None, feed=None):
        self.go(addr)
        return self.wait_report(timeout, echo, feed)

    def load_and_run(self, data, addr=0, **kw):
        self.load(addr, data)
        return self.run(addr, **kw)

    def registers(self):
        """x0..x31 as saved by the ROM trap handler at the last trap."""
        regs = self.read_words(TRAP_FRAME, 32)
        regs[0] = 0
        return regs


def main():
    import argparse
    ap = argparse.ArgumentParser(description="Load and run a program through the boot ROM")
    ap.add_argument("binary", nargs="?", help="program linked for RAM (firmware/build/*.bin)")
    ap.add_argument("-p", "--port", default="/dev/ttyUSB1")
    ap.add_argument("-a", "--addr", type=lambda s: int(s, 0), default=0)
    ap.add_argument("-t", "--timeout", type=float, default=60.0)
    ap.add_argument("--term", action="store_true", help="reset and attach a terminal (monitor)")
    args = ap.parse_args()

    board = Board(args.port)
    if args.term or not args.binary:
        board.ser.send_break(0.05)
        terminal(board.ser)
        return
    board.reset()
    data = open(args.binary, "rb").read()
    board.load(args.addr, data)
    print(f"loaded {len(data)} bytes at {args.addr:#x}, running", file=sys.stderr)
    result = board.run(args.addr, timeout=args.timeout,
                       echo=lambda d: (sys.stdout.write(d.decode("latin1")), sys.stdout.flush()))
    if isinstance(result, Trap):
        print(f"\n{result}", file=sys.stderr)
        sys.exit(1)
    print(f"\nexit code {result.code}", file=sys.stderr)


def terminal(ser):
    """Minimal terminal: stdin lines go to the board, output comes back. Ctrl-C quits."""
    import threading
    ser.timeout = 0.05

    def pump():
        while True:
            data = ser.read(256)
            if data:
                sys.stdout.write(data.decode("latin1"))
                sys.stdout.flush()

    threading.Thread(target=pump, daemon=True).start()
    try:
        for line in sys.stdin:
            ser.write(line.rstrip("\n").encode() + b"\r")
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
