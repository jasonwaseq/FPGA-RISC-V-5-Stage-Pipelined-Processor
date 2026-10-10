# RISC-V 5-Stage Pipelined Processor Makefile

# Directories
RTL_DIR    = rtl
TEST_DIR   = test
SYNTH_DIR  = synth
BUILD_DIR  = build

# Source files
RTL_FILES  = $(RTL_DIR)/riscv_core.sv \
             $(RTL_DIR)/memory.sv \
             $(RTL_DIR)/riscv_top.sv \
             $(RTL_DIR)/uart.v \
             $(RTL_DIR)/uart_tx.v \
             $(RTL_DIR)/uart_rx.v

# Top module
TOP_MODULE = riscv_top

# FPGA settings (iCEBreaker - iCE40UP5K)
DEVICE     = up5k
PACKAGE    = sg48
PCF_FILE   = $(SYNTH_DIR)/icebreaker.pcf
FREQ       = 12

#-------------------------------------------------------------------------------
# Simulation (cocotb + Icarus Verilog)
#-------------------------------------------------------------------------------
.PHONY: sim sim-fwd sim-top
# Full suite on the forwarding and interlock-only builds + CPI comparison
sim:
	python3 $(TEST_DIR)/run_tests.py

# Forwarding build only (faster)
sim-fwd:
	python3 $(TEST_DIR)/run_tests.py --forwarding 1

# Full system: riscv_top over UART, driven by scripts/test_fpga.py (slow)
sim-top:
	python3 $(TEST_DIR)/run_tests.py --top

#-------------------------------------------------------------------------------
# Synthesis (Yosys + nextpnr for iCE40)
#-------------------------------------------------------------------------------
.PHONY: build synth place route prog

# Synthesize to JSON
$(BUILD_DIR)/$(TOP_MODULE).json: $(RTL_FILES)
	@mkdir -p $(BUILD_DIR)
	yosys -p "read_verilog -sv $(RTL_FILES); \
	          synth_ice40 -top $(TOP_MODULE) -json $@"

# Place and route
$(BUILD_DIR)/$(TOP_MODULE).asc: $(BUILD_DIR)/$(TOP_MODULE).json $(PCF_FILE)
	nextpnr-ice40 --$(DEVICE) --package $(PACKAGE) \
		--json $(BUILD_DIR)/$(TOP_MODULE).json \
		--pcf $(PCF_FILE) \
		--asc $@ \
		--freq $(FREQ)

# Generate bitstream
$(BUILD_DIR)/$(TOP_MODULE).bin: $(BUILD_DIR)/$(TOP_MODULE).asc
	icepack $< $@

synth: $(BUILD_DIR)/$(TOP_MODULE).json
place: $(BUILD_DIR)/$(TOP_MODULE).asc
build: $(BUILD_DIR)/$(TOP_MODULE).bin

# Program FPGA
prog: $(BUILD_DIR)/$(TOP_MODULE).bin
	iceprog $<

# Program to flash
prog-flash: $(BUILD_DIR)/$(TOP_MODULE).bin
	iceprog $<

#-------------------------------------------------------------------------------
# FPGA Testing
#-------------------------------------------------------------------------------
.PHONY: test-fpga
test-fpga:
	python3 scripts/test_fpga.py

.PHONY: test-fpga-interactive
test-fpga-interactive:
	python3 scripts/test_fpga.py -i

#-------------------------------------------------------------------------------
# Linting
#-------------------------------------------------------------------------------
.PHONY: lint
lint:
	verilator --lint-only -Wall -sv $(RTL_FILES)

#-------------------------------------------------------------------------------
# Clean
#-------------------------------------------------------------------------------
.PHONY: clean
clean:
	rm -rf $(BUILD_DIR)/sim_* $(BUILD_DIR)/cpi_*.json
	rm -f *.vcd *.fst
	rm -rf __pycache__ $(TEST_DIR)/__pycache__
	rm -rf sim_build results.xml $(TEST_DIR)/results.xml

.PHONY: clean-all
clean-all: clean
	rm -rf .cocotb

#-------------------------------------------------------------------------------
# Help
#-------------------------------------------------------------------------------
.PHONY: help
help:
	@echo "RISC-V 5-Stage Pipelined Processor"
	@echo ""
	@echo "Simulation:"
	@echo "  make sim              - Run cocotb suite (forwarding + interlock builds, CPI table)"
	@echo "  make sim-fwd          - Run cocotb suite on the forwarding build only"
	@echo "  make sim-top          - Run scripts/test_fpga.py against the simulated FPGA"
	@echo ""
	@echo "Synthesis:"
	@echo "  make build            - Build bitstream for iCEBreaker"
	@echo "  make prog             - Program FPGA"
	@echo ""
	@echo "Testing:"
	@echo "  make test-fpga        - Run FPGA tests via UART"
	@echo "  make test-fpga-interactive - Interactive FPGA testing"
	@echo ""
	@echo "Other:"
	@echo "  make lint             - Lint RTL with Verilator"
	@echo "  make clean            - Clean build artifacts"
