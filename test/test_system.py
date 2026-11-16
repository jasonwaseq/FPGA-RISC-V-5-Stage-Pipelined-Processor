"""Full-system simulation: boot ROM, UART loader, SPI flash boot, firmware.

The SoC boots its real ROM image. The host side is scripts/rvhost.py, the same
code that drives the board, talking through a simulated serial port.
"""
import os
import struct
import sys
import zlib
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.task import bridge
from cocotb.triggers import ClockCycles

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import rvhost       # noqa: E402
import mkimage      # noqa: E402
from simserial import SimSerial     # noqa: E402

FW = ROOT / "firmware" / "build"
BIT_CLOCKS = 8                      # tb_soc: CLK_HZ / BAUD
FLASH_IMAGE_OFFSET = 0x100000


def firmware(name):
    path = FW / name
    assert path.exists(), f"{path} missing: run `make firmware` first"
    return path.read_bytes()


async def start(dut, flash_image=None):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.uart_rx.value = 1
    dut.gpio_in.value = 0
    dut.rst.value = 1
    if flash_image:
        for i, byte in enumerate(flash_image):
            dut.u_flash.mem[FLASH_IMAGE_OFFSET + i].value = byte
    await ClockCycles(dut.clk, 5)
    dut.rst.value = 0
    return rvhost.Board(serial_port=SimSerial(dut, BIT_CLOCKS))


@cocotb.test()
async def test_loader_runs_program(dut):
    """Claim the ROM loader, upload a C program, run it, get output and exit code."""
    board = await start(dut)
    program = firmware("hello.bin")

    @bridge
    def host():
        board.sync()
        board.load(0, program)
        assert board.read(0, 16) == program[:16]
        return board.run(0, timeout=3600)

    result = await host()
    dut._log.info(f"program output: {result.output.decode()!r}")
    assert isinstance(result, rvhost.Exit), result
    assert result.code == 42
    assert b"Hello from RV32IM" in result.output
    assert b"sum of squares 1..1000 = 333833500" in result.output


@cocotb.test()
async def test_trap_report(dut):
    """Unhandled traps reach the ROM, which reports them and saves the registers."""
    board = await start(dut)

    def asm(*words):
        return struct.pack(f"<{len(words)}I", *words)

    cases = {
        # li a0, 0x55 ; li t0, 0x40000000 ; lw a1, 0(t0)   -> load access fault
        "load fault": (asm(0x05500513, 0x400002B7, 0x0002A583), 5, 8, 0x40000000),
        # li s1, 77 ; .word 0xFFFFFFFF                      -> illegal instruction
        "illegal": (asm(0x04D00493, 0xFFFFFFFF), 2, 4, 0),
        # li a2, 3 ; sw a2, 2(zero)                         -> store misaligned
        "store misaligned": (asm(0x00300613, 0x00C02123), 6, 4, 2),
        # ebreak
        "breakpoint": (asm(0x00100073), 3, 0, 0),
        # li t1, 0x02000000 ; jr t1                         -> fetch outside memory
        "fetch fault": (asm(0x02000337, 0x00030067), 1, 0x02000000, 0x02000000),
    }

    @bridge
    def host():
        board.sync()
        results = {}
        for name, (code, *_rest) in cases.items():
            board.load(0x400, code)
            results[name] = (board.run(0x400, timeout=3600), board.registers())
        return results

    results = await host()
    for name, (code, cause, epc_off, tval) in cases.items():
        trap, regs = results[name]
        assert isinstance(trap, rvhost.Trap), (name, trap)
        epc = epc_off if epc_off >= 0x1000 else 0x400 + epc_off
        assert (trap.cause, trap.epc, trap.tval) == (cause, epc, tval), (name, str(trap))
        dut._log.info(f"{name}: {trap}")
    assert results["load fault"][1][10] == 0x55             # a0 in the trap frame
    assert results["load fault"][1][5] == 0x40000000        # t0
    assert results["illegal"][1][9] == 77                   # s1


@cocotb.test()
async def test_flash_boot_copy(dut):
    """With no host on the line, the ROM copies an image from SPI flash and runs it."""
    image = mkimage.boot_image(firmware("hello.bin"), load=0, entry=0)
    board = await start(dut, flash_image=image)

    @bridge
    def host():
        return board.wait_report(timeout=3600)

    result = await host()
    assert isinstance(result, rvhost.Exit) and result.code == 42, result
    assert b"RV32IM SoC boot ROM" in result.output and b"Hello from RV32IM" in result.output


@cocotb.test()
async def test_flash_boot_bad_crc(dut):
    """A corrupted image is rejected and the ROM falls back to the loader."""
    image = bytearray(mkimage.boot_image(firmware("hello.bin"), load=0, entry=0))
    image[200] ^= 0x10
    board = await start(dut, flash_image=bytes(image))

    @bridge
    def host():
        old = board.ser.timeout
        board.ser.timeout = 15
        text = bytearray()
        while not text.endswith(b"corrupt\r\n"):
            byte = board.ser.read(1)
            assert byte, f"no rejection message, got {bytes(text)!r}"
            text += byte
        board.ser.timeout = old
        board.sync()                    # the loader is alive
        return bytes(text)

    text = await host()
    assert b"boot image corrupt" in text


@cocotb.test()
async def test_monitor_xip(dut):
    """The monitor boots and runs in place from SPI flash; drive a few commands."""
    image = firmware("monitor.img")
    board = await start(dut, flash_image=image)
    payload = bytes(range(256)) * 2

    @bridge
    def host():
        ser = board.ser

        def until(marker, timeout=15):
            ser.timeout = timeout
            text = bytearray()
            while not text.endswith(marker):
                byte = ser.read(1)
                assert byte, f"timeout waiting for {marker!r}, got {bytes(text)!r}"
                text += byte
            return bytes(text)

        def command(line):
            ser.write(line.encode() + b"\r")
            return until(b"> ").decode()

        log = [until(b"> ").decode()]
        log.append(command("info"))
        log.append(command("mw 0x1000 0xCAFEF00D"))
        log.append(command("md 0x1000 1"))
        ser.write(b"load 0x2000 512\r")
        until(b"ready\r\n")
        ser.write(payload)
        log.append(until(b"> ").decode())
        log.append(command("crc 0x2000 512"))
        log.append(command("perf"))
        return log

    log = await host()
    for entry in log:
        dut._log.info(entry.strip())
    text = "".join(log)
    assert "RV32IM" in log[0]
    assert "cafef00d" in log[3].lower()
    assert f"{zlib.crc32(payload) & 0xFFFFFFFF:08x}" in log[5].lower()


@cocotb.test()
async def test_interrupts(dut):
    """Timer, software and UART receive interrupts, taken and returned from."""
    board = await start(dut)
    program = firmware("irqtest.bin")

    def feed(output):
        return b"hello" if output.endswith(b"send\r\n") else None

    @bridge
    def host():
        board.sync()
        board.load(0, program)
        return board.run(0, timeout=3600, feed=feed)

    result = await host()
    dut._log.info(result.output.decode())
    assert isinstance(result, rvhost.Exit) and result.code == 0, result


@cocotb.test()
async def test_data_tasks(dut):
    """The host supplies data, the CPU processes it, the host checks the answer."""
    import random
    board = await start(dut)
    program = firmware("tasks.bin")
    rng = random.Random(5)
    values = [rng.getrandbits(32) for _ in range(64)]
    blob = struct.pack("<64I", *values)
    DATA = 0x10000

    @bridge
    def host():
        board.sync()
        board.load(0, program)
        board.load(DATA, blob)
        board.write_words(DATA - 16, [1, DATA, len(values), 0])     # task 1: sort
        r1 = board.run(0, timeout=3600)
        sorted_words = board.read_words(DATA, len(values))
        board.load(DATA, blob)
        board.write_words(DATA - 16, [2, DATA, len(blob), 0])       # task 2: CRC-32
        r2 = board.run(0, timeout=3600)
        return r1, sorted_words, r2, board.read_words(DATA - 4, 1)[0]

    r1, sorted_words, r2, crc = await host()
    assert isinstance(r1, rvhost.Exit) and r1.code == 0, r1
    assert sorted_words == sorted(values)
    assert isinstance(r2, rvhost.Exit) and r2.code == 0, r2
    assert crc == zlib.crc32(blob) & 0xFFFFFFFF
