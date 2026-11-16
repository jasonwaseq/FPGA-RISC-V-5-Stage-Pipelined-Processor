#!/usr/bin/env bash
# Gate-level smoke test: synthesise the SoC exactly as for the FPGA, then
# simulate the resulting netlist of iCE40 primitives booting the ROM.
set -euo pipefail
cd "$(dirname "$0")/.."

# Prefer an OSS CAD Suite unpacked in the home directory over older system tools.
if [ -d "$HOME/oss-cad-suite/bin" ]; then PATH="$HOME/oss-cad-suite/bin:$PATH"; fi

RTL="rtl/riscv_core.sv rtl/icache.sv rtl/muldiv.sv rtl/csr.sv rtl/memory.sv \
     rtl/spi_flash.sv rtl/periph.sv rtl/soc.sv"
CELLS=$(yosys-config --datdir)/ice40/cells_sim.v

mkdir -p build
yosys -q -p "
    read_verilog -sv $RTL;
    chparam -set CLK_HZ 80000 -set BAUD 10000 -set FLASH_WAKE_CYCLES 16 soc;
    synth_ice40 -dsp -abc9 -top soc;
    write_verilog -noattr build/soc_netlist.v"

# The flash model lives in tb_soc.sv next to a wrapper that is not used here.
sed -n '/^module spiflash_model/,/^endmodule/p' test/tb_soc.sv > build/spiflash_model.v

iverilog -g2012 -DNO_ICE40_DEFAULT_ASSIGNMENTS -o build/netlist.vvp -s tb_netlist \
    test/tb_netlist.sv build/soc_netlist.v build/spiflash_model.v "$CELLS"
vvp -n build/netlist.vvp | tail -8
