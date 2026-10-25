"""Full-system simulation: riscv_top driven over its real UART at 115200 baud.

The host script scripts/test_fpga.py is run unmodified against the
simulated FPGA by replacing its serial port with a UART bit-banger, so the
command protocol, the counter readout and the hardware test programs are all
checked before going to the board.
"""
import os
import sys
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.task import bridge, resume
from cocotb.triggers import ClockCycles, FallingEdge, RisingEdge

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))
import test_fpga  # noqa: E402

BIT = 13 * 8          # clocks per UART bit (UART_PRESCALE = 13, 8x oversampling)


class SimSerial:
    """Drop-in for serial.Serial used by RISCVTester."""

    def __init__(self, dut):
        self.dut = dut
        self.rx = []
        cocotb.start_soon(self._monitor())

    async def _monitor(self):
        dut = self.dut
        while True:
            await FallingEdge(dut.uart_tx)
            await ClockCycles(dut.clk, BIT + BIT // 2)
            byte = 0
            for i in range(8):
                byte |= int(dut.uart_tx.value) << i
                await ClockCycles(dut.clk, BIT)
            self.rx.append(byte)

    async def _send(self, data):
        dut = self.dut
        for byte in data:
            for bit in [0] + [(byte >> i) & 1 for i in range(8)] + [1]:
                dut.uart_rx.value = bit
                await ClockCycles(dut.clk, BIT)

    async def _recv(self, n, timeout_bytes=64):
        waited = 0
        while len(self.rx) < n and waited < timeout_bytes * 10 * BIT:
            await ClockCycles(self.dut.clk, BIT)
            waited += BIT
        out, self.rx = self.rx[:n], self.rx[n:]
        return bytes(out)

    def write(self, data):
        resume(self._send)(bytes(data))

    def read(self, n):
        return resume(self._recv)(n)

    def reset_input_buffer(self):
        self.rx.clear()

    def reset_output_buffer(self):
        pass

    def close(self):
        pass


@cocotb.test()
async def test_host_script_against_sim(dut):
    cocotb.start_soon(Clock(dut.clk, 83334, unit="ps").start())   # 12 MHz
    dut.uart_rx.value = 1
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 20)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 20)

    tester = test_fpga.RISCVTester.__new__(test_fpga.RISCVTester)
    tester.ser = SimSerial(dut)
    tester.program = []

    count = int(os.environ.get("TOP_RANDOM_PROGRAMS", "2"))

    @bridge
    def host():
        results = {
            "data_forwarding": test_fpga.test_data_forwarding(tester),
            "control_flow": test_fpga.test_control_flow(tester),
            "random_vs_model": test_fpga.test_random_vs_model(tester, count=count),
            "cpi": test_fpga.test_cpi(tester),
        }
        return results

    results = await host()
    dut._log.info(f"host-script results: {results}")
    assert all(results.values()), results
