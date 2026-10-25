# FPGA RISC-V 5-Stage Pipelined Processor

An RV32I 5-stage in-order pipelined processor in SystemVerilog, running on the iCEBreaker (Lattice iCE40UP5K) FPGA. It implements the full RV32I base integer ISA with a bypass network, a load-use interlock and branch/jump flush. A UART debug interface loads programs, reads registers and reads on-chip performance counters. The design is verified with cocotb against an instruction-set reference model, and CPI is measured with and without forwarding.

## Results

| Metric | Value |
|---|---|
| ISA | RV32I base (all 37 user-level integer instructions + EBREAK/ECALL halt) |
| Pipeline | F / D / X / M / W, in-order, single issue, static not-taken prediction |
| Hazard cost | Dependent ALU ops: **0 stalls** · Load-use: **1 cycle** · Taken branch/jump: **2 cycles** |
| Area (iCE40UP5K) | 4,938 / 5,280 logic cells (93%), 4 / 30 BRAM |
| Timing | Meets the 12 MHz board clock; Fmax 14.1–14.8 MHz across placement seeds |
| Verification | 7 directed suites + 1,000 constrained-random programs per build vs. reference model; cycle-exact hazard-penalty checks; full-system UART test |

### CPI with and without forwarding

Same RTL; the `FORWARDING` parameter swaps the bypass network for a pure interlock (stall until the producer writes back). Cycle counts come from the on-chip performance counters.

| Benchmark | Instructions | CPI (forwarding) | CPI (interlock only) | Speedup |
|---|---:|---:|---:|---:|
| Fibonacci (24 terms) | 173 | 1.289 | 1.711 | 1.33× |
| Array fill + sum (64 words) | 519 | 1.617 | 2.237 | 1.38× |
| Bubble sort (16 words) | 1,088 | 1.360 | 1.764 | 1.30× |
| Dependent ALU chain | 202 | 1.020 | 3.000 | 2.94× |

Reproduce with `make sim`.

## Microarchitecture

```
          +-------+     +-------+     +-------+     +-------+     +-------+
  imem -->|   F   |---->|   D   |---->|   X   |---->|   M   |---->|   W   |
 (BRAM)   +-------+ IF/ +-------+ ID/ +-------+ EX/ +-------+ MEM/+-------+
              ^     ID      |     EX   ^  |  ^   MEM    |     WB      |
              |             |          |  |  +--EX/MEM--+             |
              |  redirect   |          |  +-----MEM/WB----------------+
              +-------------------------  (forwarding into X)
                            ^
                            +---- W -> D register-file write-through
```

- **Fetch.** The instruction memory is a synchronous iCE40 BRAM, so it is addressed with the *next* PC. The instruction for `f_pc` is on the bus during F. On a stall, the address is held and the BRAM re-reads the same word.
- **Decode.** Generates immediates for all formats and the control signals. The register file has two read ports with W→D write-through. Reads of `x0` never cause a hazard.
- **Execute.** The ALU, the branch comparator and the jump-target adders. Operands come from a forwarding mux with priority EX/MEM > MEM/WB > register file. Branches and jumps resolve here: a taken branch redirects fetch and squashes the two younger instructions.
- **Memory.** Byte enables and data steering for SB/SH/SW. The data-memory BRAM is addressed in M.
- **Writeback.** The load data arrives from the BRAM in W and is sign- or zero-extended for LB/LH/LBU/LHU. It is forwarded straight from W to X, which is why load-use costs only one stall.
- **Hazard unit.** With forwarding, only a load followed immediately by a consumer stalls (1 cycle). With `FORWARDING=0`, any RAW dependence on an instruction in X or M stalls.
- **Halt and counters.** EBREAK/ECALL squashes younger instructions, stops fetch, drains the pipeline and freezes the core. `perf_cycles` and `perf_instret` stop at the same time, so CPI = cycles / instret for exactly the program that ran.

The critical path runs from the data-memory BRAM output, through load alignment and the forwarding mux, into the branch comparator and on to the fetch address. That is the cost of resolving branches in X with full forwarding.

## Verification

`test/test_riscv.py` (cocotb + Icarus Verilog) runs the core with the real synchronous memories (`test/tb_core.sv`). Every program runs back-to-back with **no NOP padding**:

| Suite | What it checks |
|---|---|
| `test_forwarding_chain`, `test_forwarding_paths` | EX/MEM and MEM/WB paths on both operands, youngest-producer priority, x0, store-data forwarding |
| `test_load_use` | Load followed by ALU, store-data, address and branch consumers |
| `test_branches`, `test_jumps` | All six branch conditions taken and not taken; JAL/JALR links, targets, bit-0 clearing, call/return |
| `test_upper_immediates`, `test_byte_half` | LUI/AUIPC; SB/SH/LB/LBU/LH/LHU at every byte offset |
| `test_timing` | Cycle-exact penalties: 0 for dependent ALU ops, 1 for load-use, 2 for a taken branch |
| `test_random_programs` | 1,000 constrained-random programs (dense hazards, forward branches, JAL/JALR, sub-word memory ops); registers, memory and retired-instruction count all compared against `test/rv32i.py` |
| `test_benchmarks` | Benchmarks checked against the model, with CPI recorded |

The whole suite runs on both the forwarding and the interlock-only builds. The tests were also checked against a deliberately broken core with one forwarding path removed: the directed, random and benchmark suites all fail on it.

`test/test_top.py` simulates the full FPGA top level (core, BRAMs and UART at 115200 baud) and drives it with the **unmodified** host script `scripts/test_fpga.py` through a simulated serial port. This covers the command protocol and the counter readout before anything goes on the board.

## Running

```bash
# Simulation (requires cocotb >= 2.0 and Icarus Verilog)
make sim        # both builds + CPI comparison table
make sim-fwd    # forwarding build only
make sim-top    # full system over UART (a few minutes)

# FPGA (Yosys + nextpnr-ice40 + icestorm)
make build
make prog

# Hardware tests over UART (pyserial)
python3 scripts/test_fpga.py              # full suite, including random programs vs. model and CPI
python3 scripts/test_fpga.py -t cpi       # benchmarks + on-chip counters
python3 scripts/test_fpga.py -i           # interactive assembler / debugger
```

## UART Protocol (115200 baud, 8N1)

| Command | Format | Description |
|---|---|---|
| Write instruction | `0xAA <addr_hi> <addr_lo> <d3> <d2> <d1> <d0>` | Write a 32-bit word to instruction memory |
| Read register | `0xBB <reg>` | Returns 4 bytes (valid while halted) |
| Read counter | `0xBC <idx>` | Returns 4 bytes: 0 = cycles, 1 = instructions retired, 2 = halted flag |
| Run | `0xCC` | Release the pipeline from reset |
| Halt | `0xDD` | Hold the pipeline in reset (registers are kept) |
| Reset | `0xEE` | Reset the pipeline and clear the registers |

LED 0 shows that the processor is running, and LED 4 lights when the program has reached EBREAK.

## Project Structure

```
rtl/
  riscv_core.sv     5-stage core: forwarding, hazard unit, branch/jump, counters
  riscv_top.sv      iCEBreaker top: BRAMs, UART command FSM
  memory.sv         Synchronous BRAM model (byte enables)
  uart*.v           UART
test/
  tb_core.sv        Core + memories for simulation
  test_riscv.py     Directed, timing, random and benchmark tests
  test_top.py       Full-system UART test driving scripts/test_fpga.py
  rv32i.py          Encoders + instruction-set reference model
  programs.py       Benchmark programs
  run_tests.py      cocotb runner (both builds, CPI summary)
scripts/
  test_fpga.py      Hardware test suite and interactive mode
synth/icebreaker.pcf
```

## License

MIT License
