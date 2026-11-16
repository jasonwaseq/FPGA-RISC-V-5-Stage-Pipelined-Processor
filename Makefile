# RV32IM SoC for the iCEBreaker: simulation, firmware, bitstream, hardware tests.
#
# Tools: Yosys, nextpnr-ice40 and icestorm (OSS CAD Suite), Icarus Verilog and
# cocotb for simulation, a RISC-V GCC (RISCV=prefix, default riscv-none-elf-)
# for the firmware, pyserial for the host scripts.

PORT     ?= /dev/ttyUSB1
PLL_DIVF ?= 47          # system clock = 12 MHz * (PLL_DIVF + 1) / 32  -> 18 MHz
SEED     ?= 2
PYTHON   ?= python3

RTL = rtl/riscv_core.sv rtl/icache.sv rtl/muldiv.sv rtl/csr.sv rtl/memory.sv \
      rtl/spi_flash.sv rtl/periph.sv rtl/soc.sv

.PHONY: help
help:
	@echo "Simulation"
	@echo "  make sim            core suite on both builds (BTB / static) + CPI table"
	@echo "  make sim-isa        official riscv-tests (rv32ui, rv32um, rv32mi)"
	@echo "  make sim-system     boot ROM, UART loader, flash boot, monitor, interrupts"
	@echo "  make sim-netlist    gate-level boot of the synthesised netlist"
	@echo "  make lint           Verilator lint"
	@echo "Firmware"
	@echo "  make firmware       boot ROM, monitor, test programs"
	@echo "  make isa coremark   riscv-tests and CoreMark (sources fetched on first use)"
	@echo "FPGA"
	@echo "  make build          synthesis, place and route -> build/soc.bin"
	@echo "  make prog           program the bitstream"
	@echo "  make flash-monitor  store the monitor in SPI flash (boots at power-up)"
	@echo "  make seeds          place-and-route seed sweep (timing spread)"
	@echo "Hardware"
	@echo "  make test-fpga      full hardware test suite over the UART"
	@echo "  make test-coremark  CoreMark on the board"
	@echo "  make interactive    assemble, run and single-step programs on the board"
	@echo "  make term           reset the board and open the monitor shell"

# ---- Simulation -----------------------------------------------------------------
.PHONY: sim sim-isa sim-system sim-netlist lint
sim:
	$(PYTHON) test/run_tests.py

sim-isa: isa
	$(PYTHON) test/run_tests.py --isa

sim-system: firmware
	$(PYTHON) test/run_tests.py --system

sim-netlist:
	bash synth/netlist_sim.sh

lint:
	verilator --lint-only -Wall -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM \
	    -Wno-PINCONNECTEMPTY -sv --top-module soc $(RTL)

# ---- Firmware ---------------------------------------------------------------------
.PHONY: firmware isa coremark
firmware:
	$(MAKE) -C firmware

isa:
	$(MAKE) -C firmware isa

coremark:
	$(MAKE) -C firmware coremark

# ---- FPGA ---------------------------------------------------------------------------
.PHONY: build prog flash-monitor seeds
build:
	bash synth/build.sh $(PLL_DIVF) $(SEED)

prog:
	iceprog build/soc.bin

flash-monitor: firmware
	iceprog -o 1M firmware/build/monitor.img

seeds:
	bash synth/seeds.sh 18

# ---- Hardware -----------------------------------------------------------------------
.PHONY: test-fpga test-coremark interactive term
test-fpga: firmware isa
	$(PYTHON) scripts/test_fpga.py $(PORT)

test-coremark: coremark
	$(PYTHON) scripts/test_fpga.py $(PORT) -t loader --coremark

interactive:
	$(PYTHON) scripts/interactive.py $(PORT)

term:
	$(PYTHON) scripts/rvhost.py -p $(PORT) --term

.PHONY: clean
clean:
	rm -rf build firmware/build test/__pycache__ scripts/__pycache__ test/results.xml
