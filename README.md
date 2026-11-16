# RV32IM SoC on an iCE40UP5K: 5-stage pipeline, I-cache, branch prediction, boot ROM, firmware

A complete RISC-V computer in a 5,280-LUT FPGA (iCEBreaker v1.1a). The core is
an RV32IM 5-stage pipeline with machine-mode traps and interrupts. Around it is
a memory system built for what this FPGA actually has: an instruction cache and
a branch target buffer in block RAM, 128 KB of single-port SPRAM as main memory,
and the 16 MB configuration flash as execute-in-place storage. A boot ROM, a C
runtime and an interactive monitor make it usable from a serial terminal.

Everything below was measured on the board.

| | |
|---|---|
| ISA | RV32IM, Zicsr, Zifencei, M-mode (precise exceptions; timer, software and external interrupts) |
| **CoreMark** | **3.11 CoreMark/MHz** (56 CoreMark at 18 MHz), validated run, 10.7 s |
| CPI | 1.15 on CoreMark; 1.05 to 1.26 on the kernels below |
| Clock | 18 MHz from the PLL; place and route reports 20 to 21.7 MHz across seeds |
| Area | 4,661 / 5,280 logic cells, 28 / 30 block RAMs, 4 / 4 SPRAM, 4 / 8 DSP |
| Memory | 128 KB RAM, 8 KB I-cache, 256-entry BTB, 2 KB boot ROM, 16 MB flash (XIP) |
| Verification | 57 / 57 official riscv-tests, 1,000 random programs per build against a reference model, full-system and gate-level simulation, and the same tests again on hardware |

The starting point was a plain RV32I pipeline with a debug FSM on the UART:
4,938 logic cells, 13.9 MHz, 1 KB of memory, no way to run compiled code.

## Architecture

```
                    +------------------ riscv_core ------------------+
                    |  F        D        X          M          W     |
   128 KB SPRAM <---+- I-cache  decode   bypass     commit     reg   |
   boot ROM     <---+  (8 KB)   regfile  ALU        load align write |
   SPI flash    <---+  BTB      (BRAM)   branch     store            |
       ^  refill    |  PC                mul (DSP)  CSR / trap       |
       |            |                    div        redirect         |
       |            +----------+------------+----------+-------------+
       |                       | load addr  | store    | slow bus
       +-----------------------+------------+          v
         RAM port: store > load > refill        ROM, flash, UART, timer, GPIO
```

### Pipeline

| Stage | What happens |
|---|---|
| F | PC, I-cache and BTB lookup. All three are block RAMs addressed with the *next* PC, so a correctly predicted taken branch costs nothing. |
| D | Decode, immediates, register file (block RAM whose read address comes straight from the I-cache output), late-result interlock, bypass selects for X. |
| X | 4:1 bypass muxes, ALU, barrel shifter, branch resolution, 16x16 partial products (DSP), iterative divider. The address adder drives the RAM directly. |
| M | Commit point: load alignment, stores, CSR access, traps, interrupts, MRET, and the redirect for a misprediction found in X. |
| W | Register write. Never stalls. |

| Event | Cost |
|---|---|
| Dependent ALU operations | 0 cycles |
| Load, MUL or CSR result used by the next instruction | 1 cycle |
| Load directly behind a store (one RAM port) | 1 cycle |
| Predicted branch or jump | 0 cycles |
| Mispredicted branch or jump, trap, MRET, FENCE.I | 3 cycles |
| MULH\* / DIV, REM | 4 / 34 cycles |
| I-cache miss | 11 cycles from RAM, about 580 from flash |

These are checked cycle-exactly in simulation (`test_timing`).

### Design decisions that came from the FPGA

On this part a LUT plus its routing costs about 3 ns and the large RAMs sit at
the edge of the die. The first version of the new core, written the textbook
way, reached 12.9 MHz. The critical path was RAM output, load alignment, bypass
mux, branch comparator, next-PC mux, I-cache address: 78 ns. The fixes are the
shape of the pipeline:

- **Nothing that leaves X is combinational.** Branches are resolved in X but
  the redirect is issued from M. That makes a misprediction cost 3 cycles
  instead of 2, which is why there is a branch target buffer: on CoreMark 91%
  of branches are predicted and cost nothing.
- **Loads are read from X.** The address adder feeds the RAM directly, the data
  is aligned in M and reaches the bypass network from a register. Stores still
  happen at commit, so the single RAM port is arbitrated: a store wins, a load
  right behind it waits one cycle.
- **Bypass selects are computed in D.** X starts with a plain 4:1 mux instead of
  register-number comparators.
- **The register file is block RAM.** That removed 992 flip-flops and two
  32:1 read muxes. A
  block RAM cannot return data written on the same edge, so one bypass in D
  covers the instruction that just left W and the X-stage network covers the
  rest.
- **The interlock uses its own shallow decode**, because it feeds the fetch
  address. It can stall an illegal instruction for a cycle it did not need; it
  never misses a real dependence.

Result: 12.9 MHz to 21 MHz, with a lower CPI than the starting design.

### Memory system

The SPRAM is single ported and the flash is serial, yet the core needs an
instruction and a data access every cycle. The I-cache is what reconciles the
two: fetch almost always hits in block RAM, which leaves the RAM port to the
data side.

- **I-cache**: 8 KB, direct mapped, 32-byte lines, tags and data in block RAM.
  A miss refills a line through a pipelined request/response port from
  whichever memory holds the code. FENCE.I flushes it.
- **Branch target buffer**: 256 entries, 2-bit counters, for branches, JAL and
  JALR. It is only a hint. X checks the direction *and* the target of every
  prediction, so a stale or aliased entry costs a misprediction and nothing
  else. That is what allows 6-bit tags and no flush when code is replaced.
- **RAM**: four SB_SPRAM256KA as two 64 KB banks. Both banks are read on every
  access and a registered bit picks the result, which keeps the bank decode off
  the address path.
- **SPI flash**: memory-mapped, read-only. A request for the next word continues
  the open read instead of sending a new command, so a cache line costs one
  command plus eight words; a repeat of the last word is answered from the
  shift register. Programs can run directly from flash.
- **Slow bus**: ROM, flash and MMIO use a request/acknowledge handshake that
  holds the pipeline. Access outside the map raises a precise access fault.

| Address | Region |
|---|---|
| `0x0000_0000` | 128 KB RAM |
| `0x0010_0000` | 2 KB boot ROM |
| `0x0100_0000` | 16 MB SPI flash (boot image at +1 MB, behind the bitstream) |
| `0x8000_0000` | UART (512-byte receive FIFO) · `0x8000_1000` timer / software interrupt · `0x8000_2000` GPIO, clock frequency, ID |

### Traps

Every exception and interrupt is taken on the instruction in M, before it
changes any state, so traps are precise. Implemented: instruction, load and
store access faults and misalignment, illegal instruction (including unknown
CSRs), ECALL, EBREAK, and the machine timer, software and external interrupts
with `mstatus`, `mie`, `mip`, `mtvec`, `mepc`, `mcause`, `mtval`, `mscratch`.
`mcycle`, `minstret` and four `mhpmcounter`s (I-cache misses, mispredictions,
branches, interlock stalls) provide the numbers in this file.

## Software

**Boot ROM** (1.2 KB, `firmware/bootrom`). After reset it listens on the UART
for half a second. If the host answers it becomes a loader (write, read, CRC,
run); otherwise it boots the image stored in flash, either copied to RAM or
executed in place, after checking its CRC. It is also the crash handler: an
unhandled trap saves all registers where the host can read them and reports
`mcause`, `mepc` and `mtval`.

A break on the UART resets the system from any state, so the host can always
take the board back without touching it.

**Monitor** (`firmware/monitor`). The interactive shell, stored in flash and
executed in place. A 100 Hz timer interrupt keeps the uptime. Excerpt from a
session on the board:

```
RV32IM SoC boot ROM

RV32IM monitor (running in place from SPI flash). Type 'help'.
> info
RV32IM_Zicsr_Zifencei, 5-stage pipeline, M-mode
clock   18.00 MHz
memory  128 KB RAM at 0x00000000, boot ROM at 0x00100000, 16 MB flash at 0x01000000
cache   8 KB direct-mapped I-cache (32-byte lines), 256-entry BTB
> bench
sieve         339304 cycles    308903 instrs  CPI 1.098  I-miss     6  mispredict   2317 (2.3% of 97752 branches)
shellsort     357914 cycles    319315 instrs  CPI 1.120  I-miss    13  mispredict  10795 (16.1% of 66851 branches)
crc32         544346 cycles    518407 instrs  CPI 1.050  I-miss     2  mispredict   8203 (10.8% of 75791 branches)
matmul         83889 cycles     67459 instrs  CPI 1.243  I-miss     8  mispredict    453 (4.9% of 9222 branches)
```

Other commands: `md`, `mw`, `load`, `crc`, `go`, `perf`, `led`, `reboot`.

**Interactive console** (`scripts/interactive.py`, `make interactive`). Type
RISC-V assembly on the host and run it on the real core: the full RV32IM set
with labels and pseudo-instructions, a dozen example programs, register and
memory inspection (RAM, ROM, flash and peripherals), and true single-stepping
on the hardware. A step plants a temporary EBREAK on each possible successor
of the current instruction and resumes through a stub that restores all 31
registers, so every instruction executes on the FPGA, not in a model.

```
rv32> example sum
rv32> step 3
  [02000] 06400513  li a0, 100
        a0/x10 = 0x00000064  100
  [02004] 00000593  li a1, 0
  [02008] 00a585b3  add a1, a1, a0
        a1/x11 = 0x00000064  100
  next: [0200c] addi a0, a0, -1
rv32> cont
  Program finished.
    a0/x10 = 0x00000000  0
    a1/x11 = 0x000013ba  5050
rv32> write 0x80002000 1          # red LED on
rv32> exec firmware/build/hello.bin
rv32> monitor                     # drop into the shell stored in flash
```

**C runtime** (`firmware/common`): start-up code, linker scripts for RAM and
for execute-in-place, `printf`, interrupt entry, counters.

**Host tool** (`scripts/rvhost.py`): `rvhost.py program.bin` uploads a program,
runs it and prints its output and exit code; `--term` opens the monitor.

## Results

### CoreMark (on the board)

```
CoreMark Size    : 666
Iterations       : 600
Compiler version : GCC15.2.0
Compiler flags   : -O2 -funroll-loops -finline-functions
seedcrc 0xe9f5  crclist 0xe714  crcmatrix 0x1fd7  crcstate 0x8e3a  crcfinal 0xbd59
Correct operation validated.

cycles        : 193090896
instructions  : 167967471
CPI           : 1.149
branches      : 29571329, mispredicted 2755392 (9.31%)
I-cache misses: 207461
interlock     : 13527200 stall cycles
CoreMark/MHz  : 3.107
```

With plain `-O2` the score is 2.94. Where the cycles above 1.0 CPI go: 7.0%
load-use and MUL-use interlocks, 4.3% mispredictions, 1.2% I-cache misses.

### What the branch target buffer buys

Same RTL with `BPRED=0` (static not-taken), from `make sim`:

| Kernel | Instructions | CPI with BTB | CPI static | Speedup |
|---|---:|---:|---:|---:|
| Fibonacci | 172 | 1.180 | 1.547 | 1.31x |
| Array fill + sum | 518 | 1.191 | 1.898 | 1.59x |
| Bubble sort | 1,087 | 1.199 | 1.513 | 1.26x |
| Dependent ALU chain | 210 | 1.252 | 1.324 | 1.06x |
| Matrix multiply | 2,436 | 1.259 | 1.508 | 1.20x |

These short runs include their cold I-cache misses. On the board, read through
`mcycle`, the same kernels come out within a few cycles of the simulation.

## Verification

| Level | What runs | Command |
|---|---|---|
| Core in the SoC | 11 directed tests, cycle-exact hazard costs, 1,000 constrained-random RV32IM programs per build compared register-by-register and word-by-word with an instruction-set model (`test/rv32i.py`). Both builds (BTB and static). | `make sim` |
| ISA | The official riscv-tests, unmodified: rv32ui, rv32um, rv32mi. 57 / 57. | `make sim-isa` |
| System | The real boot ROM image: UART loader, C programs, trap reports, flash boot (copy and XIP), CRC rejection, warm reboot, the monitor, interrupts. Driven by the same host code as the board. | `make sim-system` |
| Netlist | The synthesised iCE40 netlist (SPRAM, block RAM and DSP primitives) boots the ROM. | `make sim-netlist` |
| Hardware | All of the above again on the FPGA over the UART, plus a full 124 KB RAM test and CoreMark. | `make test-fpga` |

`python3 scripts/test_fpga.py --coremark` on the board:

| Group | Result | What it covers |
|---|---|---|
| loader | pass | 124 KB written, CRC-checked and read back through the ROM |
| isa | pass | 57 / 57 riscv-tests |
| random | pass | 200 / 200 random programs match the reference model |
| cpi | pass | kernels match the model; cycles and instructions from the CSRs |
| traps | pass | 9 exception types with exact cause, pc and `mtval` |
| c | pass | hello, memory test, timer + software + UART interrupts |
| tasks | pass | sort, CRC-32, sum, sieve, matrix product on data sent by the host |
| monitor | pass | power-up boot from flash, shell commands, benchmarks |
| coremark | pass | validated run |

Bugs worth mentioning, because of where they were found:

- **A stale prediction squashed an MMIO store.** A BTB entry left by earlier
  code predicted "taken" on a store to the UART. The misprediction redirect
  fired in the store's first cycle in M, before its bus access finished, and
  the store was lost. No core-level test had a slow access at an address with
  a stale entry; the full-system test showed it as one missing character.
  There is now a directed test (`test_stale_prediction_on_mmio_store`).
- **The boot ROM synthesised as zeros.** Yosys dropped a `$readmemh` that sat
  behind `if (INIT_FILE != "")`. RTL simulation was fine; the gate-level
  simulation was added to find this and stays in the flow.
- **Flash reads corrupted by an LED.** The monitor ran from RAM and failed
  from flash. Bisecting on the board showed that words read from flash gained
  extra 1 bits only while pin 37 (the green LED) was driven low, independent of
  the logic driving it. That pin is left unused; WP# and HOLD# are now driven
  rather than left floating.
- **`minstret` off by one.** It counted at writeback, so a CSR read saw a value
  that depended on pipeline bubbles. It now counts at commit.

## Building and running

Tools: [OSS CAD Suite](https://github.com/YosysHQ/oss-cad-suite-build) (Yosys,
nextpnr, icestorm), Icarus Verilog, cocotb 2, pyserial, and a bare-metal RISC-V
GCC such as xPack `riscv-none-elf-gcc` (only needed for firmware; the boot ROM
image is checked in).

```bash
make sim            # core suite on both builds, CPI table
make firmware isa   # boot ROM, monitor, programs, riscv-tests
make sim-isa sim-system sim-netlist

make build prog     # bitstream (18 MHz) -> board
make flash-monitor  # monitor into SPI flash at 1 MB
make test-fpga      # hardware suite
make test-coremark
make interactive    # assemble, run and single-step programs on the board
make term           # monitor shell
```

`PLL_DIVF` selects the clock (12 MHz x (DIVF + 1) / 32). The UART divider and
the firmware pick the frequency up from the build, so nothing else changes.

## Limitations

- Machine mode only; no compressed instructions, no PMP, no debug module.
- Misaligned loads and stores trap (the ISA allows it); `rv32ui/ma_data`, which
  expects them to succeed, is not run. Neither are the rv32mi tests that need
  supervisor mode, PMP or debug triggers. The list and reasons are in
  `scripts/build_isa.py`.
- `mcycle` and `minstret` are read-only. WFI is a no-op.
- JALR is predicted with the last target, so returns from a function with
  several callers mispredict; there is no return address stack.
- The data side has no cache: RAM is as fast as a cache would be, and data read
  from flash pays the serial access each time.

## Layout

```
rtl/        riscv_core.sv  pipeline            icache.sv   I-cache, BTB, register file
            csr.sv         CSRs, traps         muldiv.sv   M extension
            memory.sv      SPRAM, boot ROM     spi_flash.sv  XIP flash reader
            periph.sv      UART, timer, GPIO   soc.sv, icebreaker_top.sv
firmware/   bootrom/  common/  monitor/  apps/  coremark/  isa/
scripts/    rvhost.py (host library and CLI)   test_fpga.py (hardware suite)
            interactive.py (console)  rvasm.py (assembler / disassembler)
            mkimage.py  build_isa.py
test/       test_core.py  test_isa.py  test_system.py  rv32i.py (reference model)
synth/      build.sh  seeds.sh  netlist_sim.sh  icebreaker.pcf
```

## License

MIT. CoreMark and riscv-tests are fetched at build time and keep their own
licenses.
