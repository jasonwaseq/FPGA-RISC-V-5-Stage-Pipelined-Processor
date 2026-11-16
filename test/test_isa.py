"""The official RISC-V ISA tests (riscv-tests, rv32ui / rv32um / rv32mi) in simulation.

Each test binary is placed in the simulated RAM and run from reset through the
whole SoC. It passes when it writes 1 to its `tohost` word.
"""
import json
import os
import struct
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

ISA = Path(__file__).resolve().parent.parent / "firmware" / "build" / "isa"


@cocotb.test()
async def test_riscv_tests(dut):
    manifest = json.loads((ISA / "manifest.json").read_text())
    only = os.environ.get("ISA_TEST")
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.uart_rx.value = 1
    dut.gpio_in.value = 0
    ram = dut.u_soc.u_ram.mem

    failures, longest = [], 0
    for name, info in manifest.items():
        if only and only not in name:
            continue
        image = (ISA / info["bin"]).read_bytes()
        image += b"\0" * (-len(image) % 4)
        words = struct.unpack(f"<{len(image) // 4}I", image)

        dut.rst.value = 1
        await ClockCycles(dut.clk, 2)           # let the previous program's last store land
        for i in range(max(len(words), longest)):
            ram[i].value = words[i] if i < len(words) else 0
        longest = max(longest, len(words))
        await ClockCycles(dut.clk, 3)
        dut.rst.value = 0

        tohost = ram[info["tohost"] >> 2]
        result = 0
        for _ in range(400):
            await ClockCycles(dut.clk, 100)
            result = tohost.value.to_unsigned()
            if result:
                break
        status = "pass" if result == 1 else \
                 f"FAIL at test {result >> 1}" if result else "TIMEOUT"
        dut._log.info(f"{name:22s} {status}")
        if result != 1:
            csr = dut.u_soc.u_core.u_csr
            dut._log.info(f"    mcause={csr.mcause_code.value.to_unsigned()} "
                          f"mepc=0x{csr.mepc_q.value.to_unsigned() << 2:x} "
                          f"mtval=0x{csr.mtval_q.value.to_unsigned():x}")
        if result != 1:
            failures.append(f"{name}: {status}")

    assert not failures, failures
