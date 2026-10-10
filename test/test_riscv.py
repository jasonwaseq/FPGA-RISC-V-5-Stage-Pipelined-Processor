"""Cocotb tests for the RV32I 5-stage pipeline.

Every test runs programs back-to-back with no NOP padding, so data hazards,
load-use interlocks and branch flushes are all exercised. Results are checked
against the instruction-set reference model in rv32i.py.

The same tests run against FORWARDING=1 (bypass network) and FORWARDING=0
(interlock only); the cycle-exact timing tests only apply to FORWARDING=1.
"""
import json
import os
import random
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, Timer

from programs import BENCHMARKS
from rv32i import *  # noqa: F401,F403

MEM_WORDS = 1024
FORWARDING = int(os.environ.get("FORWARDING", "1"))
RESULTS_DIR = Path(os.environ.get("RESULTS_DIR", "."))


# -----------------------------------------------------------------------------
# Harness
# -----------------------------------------------------------------------------
_clock_task = None
_dirty = {"imem": MEM_WORDS, "dmem": MEM_WORDS}


async def run(dut, program, dmem_init=None, max_cycles=200_000):
    """Load a program, run it to EBREAK and return (regs, cycles, instret)."""
    global _clock_task
    if _clock_task is None or _clock_task.done():   # cocotb kills it between tests
        _clock_task = cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())

    dut.rst_n.value = 0
    dut.rf_rst_n.value = 0
    dut.dbg_reg_addr.value = 0

    # Clear only what the previous program could have touched
    for i in range(_dirty["imem"]):
        dut.imem.mem[i].value = 0
    for i in range(_dirty["dmem"]):
        dut.dmem.mem[i].value = 0
    for i, w in enumerate(program):
        dut.imem.mem[i].value = w
    for addr, w in (dmem_init or {}).items():
        dut.dmem.mem[addr >> 2].value = w & MASK
    _dirty["imem"] = len(program) + 4
    _dirty["dmem"] = 0x500 // 4

    await ClockCycles(dut.clk, 3)
    dut.rst_n.value = 1
    dut.rf_rst_n.value = 1

    for _ in range(max_cycles):
        await RisingEdge(dut.clk)
        if dut.halted.value == 1:
            break
    else:
        raise AssertionError("processor did not reach EBREAK")

    await Timer(1, unit="ns")
    regs = [0] * 32
    for r in range(1, 32):
        dut.dbg_reg_addr.value = r
        await Timer(1, unit="ns")
        regs[r] = dut.dbg_reg_data.value.to_unsigned()
    return regs, dut.perf_cycles.value.to_unsigned(), dut.perf_instret.value.to_unsigned()


def dmem_word(dut, addr):
    return dut.dmem.mem[addr >> 2].value.to_unsigned()


async def check_against_model(dut, program, dmem_init=None, dmem_check=256, tag=""):
    ref = ISS(program, dmem_init=dmem_init)
    ref.run()
    regs, cycles, instret = await run(dut, program, dmem_init)
    for r in range(32):
        assert regs[r] == ref.x[r], \
            f"{tag} x{r}: rtl=0x{regs[r]:08x} model=0x{ref.x[r]:08x}"
    for w in range(dmem_check):
        got, exp = dmem_word(dut, w * 4), ref.load_word(w * 4)
        assert got == exp, f"{tag} mem[0x{w*4:x}]: rtl=0x{got:08x} model=0x{exp:08x}"
    assert instret == ref.retired, f"{tag} instret rtl={instret} model={ref.retired}"
    return cycles, instret


# -----------------------------------------------------------------------------
# Directed tests
# -----------------------------------------------------------------------------
@cocotb.test()
async def test_forwarding_chain(dut):
    """Back-to-back RAW chain (the test_fpga.py program with no NOPs)."""
    prog = [ADDI(1, 0, 1), ADDI(2, 1, 1), ADDI(3, 2, 1), ADDI(4, 3, 1),
            ADDI(5, 4, 1), ADD(6, 1, 5), ADD(7, 2, 4), ADD(8, 3, 3), EBREAK()]
    regs, _, _ = await run(dut, prog)
    assert regs[1:9] == [1, 2, 3, 4, 5, 6, 6, 6], regs[1:9]


@cocotb.test()
async def test_forwarding_paths(dut):
    """EX/MEM and MEM/WB paths on both operands, priority, x0 and store data."""
    prog = [
        ADDI(1, 0, 10),
        ADDI(1, 1, 5),          # x1 = 15  (EX/MEM -> rs1)
        ADDI(2, 0, 7),
        SUB(3, 1, 2),           # x3 = 8   (MEM/WB -> rs1, EX/MEM -> rs2)
        ADDI(11, 0, 9),
        ADDI(12, 0, 30),
        SUB(13, 12, 11),        # x13 = 21 (EX/MEM -> rs1, MEM/WB -> rs2)
        ADDI(4, 0, 1),
        ADDI(4, 0, 2),          # two writers of x4: youngest must win
        ADD(5, 4, 4),           # x5 = 4
        ADDI(0, 0, 99),         # write to x0 must not forward
        ADD(6, 0, 0),           # x6 = 0
        ADDI(7, 0, 0x123),
        SW(7, 0, 0x40),         # store data forwarded (EX/MEM -> rs2)
        LW(8, 0, 0x40),
        EBREAK(),
    ]
    await check_against_model(dut, prog, tag="paths")
    assert dmem_word(dut, 0x40) == 0x123


@cocotb.test()
async def test_load_use(dut):
    """Load followed immediately by consumers of every kind."""
    prog = [
        ADDI(1, 0, 0x55), SW(1, 0, 0x10),
        LW(2, 0, 0x10), ADDI(3, 2, 1),          # ALU consumer
        LW(4, 0, 0x10), SW(4, 0, 0x14),         # store-data consumer
        LW(5, 0, 0x14), LW(6, 5, -0x41),        # address consumer (0x55-0x41 = 0x14)
        LW(7, 0, 0x10), BEQ(7, 1, 8),           # branch consumer (taken)
        ADDI(9, 0, 1),                          # skipped
        ADDI(10, 0, 2),
        EBREAK(),
    ]
    await check_against_model(dut, prog, tag="load-use")


@cocotb.test()
async def test_branches(dut):
    """All six conditions, taken and not taken, with forwarded operands."""
    prog = [ADDI(1, 0, -1), ADDI(2, 0, 1)]
    for i, fn in enumerate(BRANCHES):
        prog += [fn(1, 2, 8), ADDI(10 + i, 0, 1),     # taken or not
                 fn(2, 1, 8), ADDI(20 + i, 0, 1)]
    prog += [EBREAK()]
    await check_against_model(dut, prog, tag="branches")


@cocotb.test()
async def test_jumps(dut):
    """JAL / JALR links and targets, call/return with a forwarded return address."""
    prog = [
        JAL(1, 12),             # 0x00 -> 0x0C, x1 = 0x04
        ADDI(5, 0, 1),          # 0x04 skipped... returned to later
        EBREAK(),               # 0x08
        ADDI(6, 0, 2),          # 0x0C
        JALR(0, 1, 0),          # 0x10 return to 0x04 (x1 forwarded from MEM/WB)
    ]
    await check_against_model(dut, prog, tag="jal")

    prog = [
        AUIPC(3, 0),            # 0x00 x3 = 0
        ADDI(3, 3, 0x11),       # 0x04 x3 = 0x11 (bit 0 must be cleared)
        JALR(4, 3, 4),          # 0x08 -> 0x14, x4 = 0x0C
        ADDI(7, 0, 1),          # 0x0C skipped
        ADDI(8, 0, 1),          # 0x10 skipped
        ADDI(9, 4, 0),          # 0x14 x9 = link
        EBREAK(),
    ]
    await check_against_model(dut, prog, tag="jalr")


@cocotb.test()
async def test_upper_immediates(dut):
    prog = [LUI(1, 0xDEADB), ADDI(1, 1, -0x111), AUIPC(2, 0x1), AUIPC(3, 0xFFFFF),
            *li(4, 0x80000000), *li(5, 0x12345FFF), EBREAK()]
    await check_against_model(dut, prog, tag="upper")


@cocotb.test()
async def test_byte_half(dut):
    """Sub-word stores and sign/zero-extending loads at every offset."""
    prog = [*li(1, 0x8081_F2F3)]
    for off in range(4):
        prog += [SB(1, 0, 0x20 + off), SRLI(1, 1, 8)]
    prog += [*li(2, 0xA5B6_8C7D), SH(2, 0, 0x30), SH(2, 0, 0x36)]
    for off in range(4):
        prog += [LB(10 + off, 0, 0x20 + off), LBU(14 + off, 0, 0x20 + off)]
    prog += [LH(18, 0, 0x30), LHU(19, 0, 0x30), LH(20, 0, 0x36), LHU(21, 0, 0x36),
             LW(22, 0, 0x34), EBREAK()]
    await check_against_model(dut, prog, tag="byte/half")


# -----------------------------------------------------------------------------
# Cycle-exact timing (forwarding build only)
# -----------------------------------------------------------------------------
@cocotb.test(skip=(FORWARDING == 0))
async def test_timing(dut):
    """Dependent ALU ops cost 0 stalls, load-use costs 1, taken branch costs 2."""
    base = [ADDI(1, 0, 1), ADDI(2, 0, 2), ADD(3, 1, 1), ADD(4, 2, 2), EBREAK()]
    dep  = [ADDI(1, 0, 1), ADDI(2, 1, 2), ADD(3, 2, 1), ADD(4, 3, 2), EBREAK()]
    _, c_base, _ = await run(dut, base)
    _, c_dep, _ = await run(dut, dep)
    assert c_dep == c_base, f"dependent ALU chain stalled: {c_dep} vs {c_base}"

    lu_free = [ADDI(1, 0, 4), LW(2, 0, 0), ADDI(3, 1, 1), ADDI(4, 2, 1), EBREAK()]
    lu      = [ADDI(1, 0, 4), LW(2, 0, 0), ADDI(3, 2, 1), ADDI(4, 1, 1), EBREAK()]
    _, c_free, _ = await run(dut, lu_free)
    _, c_lu, _ = await run(dut, lu)
    assert c_lu == c_free + 1, f"load-use penalty {c_lu - c_free}, expected 1"

    nt = [ADDI(1, 0, 1), BEQ(1, 0, 8), ADDI(2, 0, 1), EBREAK()]   # not taken
    tk = [ADDI(1, 0, 1), BNE(1, 0, 4), ADDI(2, 0, 1), EBREAK()]   # taken to next
    _, c_nt, _ = await run(dut, nt)
    _, c_tk, _ = await run(dut, tk)
    assert c_tk == c_nt + 2, f"taken-branch penalty {c_tk - c_nt}, expected 2"
    dut._log.info("dependent ALU: +0 cycles, load-use: +1, taken branch: +2")


# -----------------------------------------------------------------------------
# Constrained-random programs vs. reference model
# -----------------------------------------------------------------------------
def random_program(rng, length):
    """Random RV32I code with dense hazards: few registers, forward-only control flow."""
    regs = list(range(1, 8))
    prog = [*li(9, 0x100), ADDI(1, 0, rng.randint(-2048, 2047))]
    while len(prog) < length:
        k = rng.random()
        rd, a, b = rng.choice(regs), rng.choice(regs + [0]), rng.choice(regs + [0])
        remaining = length - len(prog)
        if k < 0.30:
            prog.append(rng.choice(ALU_RR)(rd, a, b))
        elif k < 0.50:
            prog.append(rng.choice(ALU_RI)(rd, a, rng.randint(-2048, 2047)))
        elif k < 0.55:
            prog.append(rng.choice(SHIFT_I)(rd, a, rng.randint(0, 31)))
        elif k < 0.60:
            prog.append(rng.choice([LUI, AUIPC])(rd, rng.randint(0, 0xFFFFF)))
        elif k < 0.75:                      # load/store relative to x9 (fixed base)
            kind = rng.choice(["lw", "lh", "lhu", "lb", "lbu", "sw", "sh", "sb"])
            align = {"w": 4, "h": 2, "b": 1}[kind[1]]
            off = rng.randrange(0, 128, align)
            if kind == "lw":   prog.append(LW(rd, 9, off))
            elif kind == "lh":  prog.append(LH(rd, 9, off))
            elif kind == "lhu": prog.append(LHU(rd, 9, off))
            elif kind == "lb":  prog.append(LB(rd, 9, off))
            elif kind == "lbu": prog.append(LBU(rd, 9, off))
            elif kind == "sw":  prog.append(SW(a, 9, off))
            elif kind == "sh":  prog.append(SH(a, 9, off))
            else:               prog.append(SB(a, 9, off))
        elif k < 0.80:                      # computed address: forwarding into AGU
            off = rng.randrange(0, 128, 4)
            prog += [ADDI(rd, 9, off), LW(rng.choice(regs), rd, 0)]
        elif k < 0.92 and remaining > 3:    # forward branch
            prog.append(rng.choice(BRANCHES)(a, b, 4 * rng.randint(1, min(4, remaining - 1))))
        elif k < 0.96 and remaining > 3:
            prog.append(JAL(rng.choice(regs + [0]), 4 * rng.randint(1, min(4, remaining - 1))))
        elif remaining > 4:                 # JALR to a forward target
            skip = rng.randint(0, min(3, remaining - 3))
            prog += [AUIPC(8, 0), JALR(rng.choice(regs + [0]), 8, 8 + 4 * skip)]
    return prog[:length] + [EBREAK()]


@cocotb.test()
async def test_random_programs(dut):
    seed = int(os.environ.get("SEED", "2026"))
    count = int(os.environ.get("RANDOM_PROGRAMS", "1000"))
    rng = random.Random(seed)
    total = 0
    for n in range(count):
        # A branch can skip the AUIPC that feeds a JALR and create a loop;
        # keep only programs the reference model can finish.
        while True:
            prog = random_program(rng, rng.randint(20, 80))
            init = {0x100 + 4 * i: rng.getrandbits(32) for i in range(40)}
            try:
                ISS(prog, dmem_init=init).run(max_steps=2000)
                break
            except RuntimeError:
                continue
        await check_against_model(dut, prog, dmem_init=init, dmem_check=0x200 // 4,
                                  tag=f"seed={seed} prog={n}")
        total += len(prog)
    dut._log.info(f"{count} random programs ({total} instructions) matched the reference model")


# -----------------------------------------------------------------------------
# Benchmarks: correctness + CPI
# -----------------------------------------------------------------------------
@cocotb.test()
async def test_benchmarks(dut):
    results = {}
    for name, gen in BENCHMARKS.items():
        prog = gen()
        cycles, instret = await check_against_model(dut, prog, dmem_check=0x500 // 4, tag=name)
        results[name] = {"cycles": cycles, "instret": instret, "cpi": round(cycles / instret, 3)}
        dut._log.info(f"{name:16s} cycles={cycles:6d} instret={instret:6d} "
                      f"CPI={cycles / instret:.3f}")
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    (RESULTS_DIR / f"cpi_forwarding{FORWARDING}.json").write_text(json.dumps(results, indent=2))
