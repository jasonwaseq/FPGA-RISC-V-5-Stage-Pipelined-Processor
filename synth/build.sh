#!/usr/bin/env bash
# Synthesise, place and route the SoC for the iCEBreaker.
#
#   synth/build.sh [PLL_DIVF] [SEED]
#
# The system clock is 12 MHz * (PLL_DIVF + 1) / 32. Output goes to build/.
set -euo pipefail
cd "$(dirname "$0")/.."

DIVF=${1:-47}
SEED=${2:-1}
FREQ=$(python3 -c "print(12 * ($DIVF + 1) / 32)")
ROM=firmware/bootrom/bootrom.hex
RTL="rtl/riscv_core.sv rtl/icache.sv rtl/muldiv.sv rtl/csr.sv rtl/memory.sv \
     rtl/spi_flash.sv rtl/periph.sv rtl/soc.sv rtl/icebreaker_top.sv"

mkdir -p build
yosys -q -l build/yosys.log -p "
    read_verilog -sv $RTL;
    chparam -set PLL_DIVF $DIVF icebreaker_top;
    synth_ice40 -dsp -abc9 -top icebreaker_top -json build/soc.json"

nextpnr-ice40 --up5k --package sg48 --json build/soc.json --pcf synth/icebreaker.pcf \
    --asc build/soc.asc --freq "$FREQ" --seed "$SEED" --log build/nextpnr.log -q \
    || { grep -E "ERROR|Max frequency" build/nextpnr.log; exit 1; }

icepack build/soc.asc build/soc.bin

echo "clock target: $FREQ MHz"
grep -E "ICESTORM_(LC|RAM|DSP|SPRAM|PLL)" build/nextpnr.log | tail -5
grep "Max frequency" build/nextpnr.log | tail -1
