"""A serial port for the simulated SoC.

Implements the subset of pyserial that scripts/rvhost.py uses, on top of the
simulated UART pins, so the unmodified host code can drive the simulation.
The blocking host code runs in a thread (cocotb `bridge`); each call here
re-enters the simulator with `resume`.
"""
import cocotb
from cocotb.task import resume
from cocotb.triggers import ClockCycles, FallingEdge


class SimSerial:
    def __init__(self, dut, bit_clocks, clocks_per_second=200_000):
        self.dut = dut
        self.bit = bit_clocks
        self.cps = clocks_per_second        # how a host-side timeout maps to simulated time
        self.timeout = 1.0
        self.rx = bytearray()
        dut.uart_rx.value = 1
        cocotb.start_soon(self._monitor())

    async def _monitor(self):
        dut = self.dut
        while True:
            await FallingEdge(dut.uart_tx)
            await ClockCycles(dut.clk, self.bit + self.bit // 2)
            byte = 0
            for i in range(8):
                byte |= int(dut.uart_tx.value) << i
                await ClockCycles(dut.clk, self.bit)
            self.rx.append(byte)

    async def _send(self, data):
        dut = self.dut
        for byte in data:
            for bit in [0] + [(byte >> i) & 1 for i in range(8)] + [1]:
                dut.uart_rx.value = bit
                await ClockCycles(dut.clk, self.bit)

    async def _recv(self, n):
        budget = int((self.timeout or 0) * self.cps)
        while len(self.rx) < n and budget > 0:
            await ClockCycles(self.dut.clk, self.bit)
            budget -= self.bit
        out = bytes(self.rx[:n])
        del self.rx[:n]
        return out

    async def _break(self):
        self.dut.rst.value = 1
        await ClockCycles(self.dut.clk, 4)
        self.dut.rst.value = 0
        self.rx.clear()

    # ---- pyserial look-alike -------------------------------------------------
    def write(self, data):
        resume(self._send)(bytes(data))

    def read(self, n):
        return resume(self._recv)(n)

    def send_break(self, duration=0.25):
        resume(self._break)()

    def reset_input_buffer(self):
        self.rx.clear()

    def close(self):
        pass
