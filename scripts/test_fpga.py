#!/usr/bin/env python3
"""Hardware test suite for the RV32IM SoC on the iCEBreaker.

Everything goes through the boot ROM loader over the UART (scripts/rvhost.py):
programs are uploaded into RAM, run on the real core, and their results are
read back and compared with a reference computed on the host.

    python3 scripts/test_fpga.py                 # full suite
    python3 scripts/test_fpga.py -t isa random   # selected groups
    python3 scripts/test_fpga.py --coremark      # add the 10+ second CoreMark run

Groups
    loader   ROM protocol: ping, memory write / read-back / CRC over all of RAM
    isa      the official riscv-tests (rv32ui, rv32um, rv32mi), unmodified
    random   constrained-random programs vs. the instruction-set model
    cpi      benchmark kernels, timed with the on-chip performance counters
    traps    precise exceptions reported with cause, pc and register dump
    c        C programs: hello, memory test, interrupts
    tasks    host supplies data, the CPU computes, the host checks the answer
    monitor  boot from SPI flash into the execute-in-place monitor (needs
             `make flash-monitor` once)
"""
import argparse
import json
import random
import struct
import sys
import time
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "test"))
sys.path.insert(0, str(ROOT / "scripts"))

import rv32i                                    # noqa: E402
from rv32i import ISS, EBREAK, ADDI, MASK       # noqa: E402
from programs import BENCHMARKS, random_case    # noqa: E402
from rvhost import Board, Exit, Trap, ProtocolError     # noqa: E402

FW = ROOT / "firmware" / "build"
BASE = 0x2000                   # where test programs are placed (their data is below)
DATA_BYTES = 0x800              # data area used by the test programs
CAUSE_BREAKPOINT = 3


def firmware(name):
    path = FW / name
    if not path.exists():
        raise FileNotFoundError(f"{path} is missing: run `make firmware` first")
    return path.read_bytes()


def words_to_bytes(words):
    return struct.pack(f"<{len(words)}I", *[w & MASK for w in words])


def csrr(rd, csr):
    return (csr << 20) | (2 << 12) | (rd << 7) | 0x73


# -----------------------------------------------------------------------------
# Running bare instruction sequences and comparing with the model
# -----------------------------------------------------------------------------
ZERO_REGS = [ADDI(r, 0, 0) for r in range(1, 32)]


def run_words(board, program, dmem_init=None, timeout=10.0):
    """Run instruction words that end in EBREAK; return the register dump.

    The ROM's trap handler saves x1..x31 when the EBREAK traps, so the final
    register state can be read back from the trap frame.
    """
    # The model starts from zeroed memory; the board keeps whatever ran before.
    image = bytearray(DATA_BYTES)
    for addr, word in (dmem_init or {}).items():
        image[addr:addr + 4] = struct.pack("<I", word & MASK)
    board.write(0, bytes(image))
    board.load(BASE, words_to_bytes(program))
    result = board.run(BASE, timeout=timeout)
    if not isinstance(result, Trap) or result.cause != CAUSE_BREAKPOINT:
        raise AssertionError(f"program did not end at its EBREAK: {result}")
    return board.registers(), result


def compare_with_model(board, body, dmem_init=None, mem_range=(0x100, 0x200), skip_regs=()):
    """Run `body` on the board and on the model; return a list of differences."""
    program = ZERO_REGS + list(body)
    ref = ISS(program, base=BASE, dmem_init=dmem_init)
    ref.run()
    regs, _ = run_words(board, program, dmem_init)
    diffs = [f"x{r}: board {regs[r]:#010x}, model {ref.x[r]:#010x}"
             for r in range(1, 32) if r not in skip_regs and regs[r] != ref.x[r]]
    lo, hi = mem_range
    mem = board.read_words(lo, (hi - lo) // 4)
    diffs += [f"mem[{lo + 4 * i:#x}]: board {w:#010x}, model {ref.load_word(lo + 4 * i):#010x}"
              for i, w in enumerate(mem) if w != ref.load_word(lo + 4 * i)]
    return diffs, regs


# -----------------------------------------------------------------------------
# Test groups
# -----------------------------------------------------------------------------
def test_loader(board):
    """Loader protocol and a full-RAM write / read-back / CRC."""
    rng = random.Random(1)
    blob = rng.randbytes(0x1F000)                       # everything below the ROM's stack
    t0 = time.time()
    board.load(0, blob)
    dt = time.time() - t0
    back = board.read(0x8000, 4096)
    ok = back == blob[0x8000:0x9000]
    print(f"  wrote and CRC-checked {len(blob) // 1024} KB in {dt:.1f} s, read-back "
          f"{'matches' if ok else 'MISMATCH'}")
    clk = board.read_words(0x8000_2008, 2)
    print(f"  system clock {clk[0] / 1e6:.2f} MHz, id {struct.pack('>I', clk[1]).decode()}")
    return ok and clk[1] == 0x52563553


def test_isa(board):
    """riscv-tests: each binary reports through its `tohost` word."""
    manifest_path = FW / "isa" / "manifest.json"
    if not manifest_path.exists():
        print("  firmware/build/isa is missing: run `make isa` first")
        return False
    manifest = json.loads(manifest_path.read_text())
    failed = []
    for i, (name, info) in enumerate(manifest.items()):
        board.load(0, (FW / "isa" / info["bin"]).read_bytes())
        board.go(0)
        time.sleep(0.05)                # every test finishes within a few milliseconds
        board.reset()                   # the test spins on tohost; RAM survives the reset
        result = board.read_words(info["tohost"], 1)[0]
        if result != 1:
            failed.append(f"{name} (tohost={result:#x}, test {result >> 1})")
        print(f"\r  {i + 1}/{len(manifest)} {name:24s}", end="", flush=True)
    print(f"\r  {len(manifest) - len(failed)}/{len(manifest)} riscv-tests passed" + " " * 20)
    for f in failed:
        print(f"  FAILED: {f}")
    return not failed


def test_random(board, count=200, seed=2026):
    """Constrained-random RV32IM programs, compared register by register."""
    rng = random.Random(seed)
    bad = 0
    for n in range(count):
        prog, init, _ = random_case(rng, base=BASE + 4 * len(ZERO_REGS))
        diffs, _ = compare_with_model(board, prog, init)
        if diffs:
            bad += 1
            print(f"\n  program {n} (seed {seed}): " + "; ".join(diffs[:4]))
        print(f"\r  {n + 1}/{count}", end="", flush=True)
    print(f"\r  {count - bad}/{count} random programs matched the reference model")
    return bad == 0


def test_cpi(board):
    """Benchmark kernels: results vs. the model, CPI from mcycle / minstret."""
    ok = True
    print(f"  {'benchmark':16s} {'cycles':>8s} {'instrs':>8s} {'CPI':>6s}")
    for name, gen in BENCHMARKS.items():
        body = gen()
        assert body[-1] == EBREAK()
        # Read the counters around the kernel; x28-x31 are not used by it.
        timed = [csrr(30, 0xB00), csrr(31, 0xB02)] + body[:-1] + \
                [csrr(28, 0xB00), csrr(29, 0xB02), EBREAK()]
        regs, _ = run_words(board, ZERO_REGS + timed)
        cycles = (regs[28] - regs[30]) & MASK
        instrs = ((regs[29] - regs[31]) & MASK) - 2         # the two csrr in between
        ref = ISS(ZERO_REGS + body, base=BASE)
        ref.run()
        same = all(regs[r] == ref.x[r] for r in range(1, 28))
        mem = board.read_words(0, 0x500 // 4)
        same &= all(w == ref.load_word(4 * i) for i, w in enumerate(mem))
        same &= instrs == ref.retired - len(ZERO_REGS)
        ok &= same
        print(f"  {name:16s} {cycles:8d} {instrs:8d} {cycles / instrs:6.3f} "
              f"{'' if same else ' RESULT MISMATCH'}")
    return ok


def test_traps(board):
    """Each exception is taken precisely and reported with the right cause."""
    li = rv32i.li
    cases = [
        ("illegal instruction", [ADDI(9, 0, 77), 0xFFFFFFFF], 2, 4, 0),
        ("load access fault", [*li(5, 0x40000000), rv32i.LW(6, 5, 0)], 5, 8, 0x40000000),
        ("store access fault", [*li(5, 0x00100000), rv32i.SW(5, 5, 0)], 7, 8, 0x00100000),
        ("load misaligned", [ADDI(5, 0, 0x101), rv32i.LW(6, 5, 0)], 4, 4, 0x101),
        ("store misaligned", [ADDI(5, 0, 0x102), rv32i.SW(5, 5, 0)], 6, 4, 0x102),
        ("fetch access fault", [*li(6, 0x02000000), rv32i.JALR(0, 6, 0)], 1, None, 0x02000000),
        ("jump misaligned", [rv32i.JAL(1, 6)], 0, 0, 0),
        ("environment call", [ADDI(10, 0, 5), 0x00000073], 11, 4, 0),
        ("unknown CSR", [0x7FF02573], 2, 0, 0),
    ]
    ok = True
    for name, body, cause, epc_off, tval in cases:
        board.load(BASE, words_to_bytes(ZERO_REGS + body))
        result = board.run(BASE, timeout=5)
        code_base = BASE + 4 * len(ZERO_REGS)
        epc = tval if epc_off is None else code_base + epc_off
        good = isinstance(result, Trap) and (result.cause, result.epc, result.tval) == (cause, epc, tval)
        ok &= good
        print(f"  {name:22s} {'ok ' if good else 'FAIL'}  {result}")
    regs = board.registers()
    return ok and regs[5] == 0 and regs[10] == 0        # last case left the zeroed registers


def test_c(board):
    """C programs through the whole stack: crt0, libc, traps, interrupts."""
    ok = True

    r = board.load_and_run(firmware("hello.bin"), timeout=10)
    good = isinstance(r, Exit) and r.code == 42 and b"333833500" in r.output
    print(f"  hello     {'ok ' if good else 'FAIL'}  {r.output.decode(errors='replace').strip().splitlines()[-1] if r.output else r}")
    ok &= good

    r = board.load_and_run(firmware("memtest.bin"), timeout=60)
    good = isinstance(r, Exit) and r.code == 0
    print(f"  memtest   {'ok ' if good else 'FAIL'}  {r.output.decode(errors='replace').strip().replace(chr(13), '').replace(chr(10), ' | ') if isinstance(r, Exit) else r}")
    ok &= good

    board.load(0, firmware("irqtest.bin"))
    r = board.run(0, timeout=30,
                  feed=lambda out: b"hello" if out.endswith(b"send\r\n") else None)
    good = isinstance(r, Exit) and r.code == 0
    print(f"  irqtest   {'ok ' if good else 'FAIL'}  {r.output.decode(errors='replace').strip().splitlines()[-1] if r.output else r}")
    ok &= good
    return ok


def test_tasks(board):
    """The CPU as a compute server for data sent by the host."""
    rng = random.Random(7)
    DATA, REQ = 0x10000, 0xFFF0
    board.load(0, firmware("tasks.bin"))
    ok = True

    def request(task, length, timeout=30):
        board.write_words(REQ, [task, DATA, length, 0])
        t0 = time.time()
        r = board.run(0, timeout=timeout)
        if not isinstance(r, Exit) or r.code != 0:
            raise AssertionError(f"task {task}: {r}")
        return board.read_words(REQ + 12, 1)[0], time.time() - t0

    values = [rng.getrandbits(32) for _ in range(4096)]
    board.load(DATA, words_to_bytes(values))
    _, dt = request(1, len(values))
    good = board.read_words(DATA, len(values)) == sorted(values)
    print(f"  sort 4096 words          {'ok ' if good else 'FAIL'}  {dt * 1000:.0f} ms")
    ok &= good

    blob = rng.randbytes(32768)
    board.load(DATA, blob)
    crc, dt = request(2, len(blob))
    good = crc == zlib.crc32(blob) & MASK
    print(f"  CRC-32 of 32 KB          {'ok ' if good else 'FAIL'}  {crc:08x}  {dt * 1000:.0f} ms")
    ok &= good

    total, dt = request(3, len(blob) // 4)
    good = total == sum(struct.unpack(f"<{len(blob) // 4}I", blob)) & MASK
    print(f"  sum of 8192 words        {'ok ' if good else 'FAIL'}  {total:08x}")
    ok &= good

    n = 30000
    primes, dt = request(4, n)
    sieve = bytearray([1]) * n
    want = 0
    for i in range(2, n):
        if sieve[i]:
            want += 1
            sieve[i * i::i] = bytes(len(sieve[i * i::i]))
    good = primes == want
    print(f"  primes below {n}       {'ok ' if good else 'FAIL'}  {primes}  {dt * 1000:.0f} ms")
    ok &= good

    n = 24
    a = [rng.randint(-1000, 1000) for _ in range(n * n)]
    b = [rng.randint(-1000, 1000) for _ in range(n * n)]
    board.load(DATA, words_to_bytes(a + b))
    _, dt = request(5, n)
    c = board.read_words(DATA + 8 * n * n, n * n)
    want = [sum(a[i * n + k] * b[k * n + j] for k in range(n)) & MASK
            for i in range(n) for j in range(n)]
    good = c == want
    print(f"  {n}x{n} matrix product     {'ok ' if good else 'FAIL'}  {dt * 1000:.0f} ms")
    ok &= good
    return ok


def test_coremark(board):
    data = firmware("coremark.bin")
    board.load(0, data)
    print(f"  loaded {len(data)} bytes, running (takes more than 10 seconds)")
    r = board.run(0, timeout=120)
    text = r.output.decode(errors="replace").replace("\r", "")
    for line in text.strip().splitlines():
        print("    " + line)
    return isinstance(r, Exit) and "Correct operation validated" in text


def test_monitor(board):
    """Power-up path: ROM -> SPI flash -> monitor running in place from flash."""
    ser = board.ser
    ser.send_break(0.05)                 # reset, and this time leave the ROM alone

    def until(marker, timeout=5.0):
        ser.timeout = 0.05
        text, deadline = bytearray(), time.time() + timeout
        while not text.endswith(marker):
            text += ser.read(256)
            if time.time() > deadline:
                raise ProtocolError(f"timeout waiting for {marker!r}; got {bytes(text)!r}")
        return bytes(text).decode(errors="replace").replace("\r", "")

    def command(line, timeout=20.0):
        ser.write(line.encode() + b"\r")
        return until(b"> ", timeout)

    try:
        banner = until(b"> ")
    except ProtocolError as e:
        if b"no boot image" in str(e).encode():
            print("  no monitor image in flash: run `make flash-monitor` first")
        else:
            print(f"  {e}")
        board.reset()
        return False
    print("  " + banner.strip().replace("\n", "\n  "))
    for line in command("info").strip().splitlines()[1:-1]:
        print("    " + line)
    payload = random.Random(3).randbytes(2048)
    ser.write(b"load 0x2000 2048\r")
    until(b"ready\r\n")
    ser.write(payload)
    loaded = until(b"> ")
    ok = f"{zlib.crc32(payload) & MASK:08x}" in loaded
    print(f"    load + crc over the shell: {'ok' if ok else 'FAIL'}")
    bench = command("bench", timeout=60)
    for line in bench.strip().splitlines()[1:-1]:
        print("    " + line)
    ok &= "result 000008d6" in bench            # 2262 primes below 20000
    board.reset()
    return ok


GROUPS = {
    "loader": test_loader, "isa": test_isa, "random": test_random, "cpi": test_cpi,
    "traps": test_traps, "c": test_c, "tasks": test_tasks, "monitor": test_monitor,
}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("port", nargs="?", default="/dev/ttyUSB1")
    ap.add_argument("-t", "--test", nargs="+", choices=list(GROUPS), help="groups to run")
    ap.add_argument("--coremark", action="store_true", help="also run CoreMark")
    ap.add_argument("--random", type=int, default=200, help="number of random programs")
    args = ap.parse_args()

    board = Board(args.port)
    board.reset()
    print(f"connected to the boot ROM loader on {args.port}")

    groups = dict(GROUPS)
    groups["random"] = lambda b: test_random(b, count=args.random)
    selected = args.test or list(GROUPS)
    if args.coremark:
        groups["coremark"] = test_coremark
        selected = selected + ["coremark"]

    results = {}
    for name in selected:
        print(f"\n[{name}] {(GROUPS.get(name) or test_coremark).__doc__ or 'CoreMark'}".rstrip())
        try:
            results[name] = bool(groups[name](board))
        except (ProtocolError, AssertionError, FileNotFoundError) as e:
            print(f"  ERROR: {e}")
            results[name] = False
            try:
                board.reset()
            except ProtocolError:
                pass

    print("\n" + "=" * 46)
    for name, ok in results.items():
        print(f"  {name:10s} {'PASS' if ok else 'FAIL'}")
    print("=" * 46)
    print("ALL HARDWARE TESTS PASSED" if all(results.values()) else "SOME TESTS FAILED")
    sys.exit(0 if all(results.values()) else 1)


if __name__ == "__main__":
    main()
