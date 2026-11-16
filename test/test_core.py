"""Cocotb tests for the RV32IM pipeline inside the full SoC.

Programs are written straight into the simulated main memory and executed
through the real memory system (I-cache refills, branch target buffer, the
single-ported RAM shared between fetch and data). Every program runs with no
NOP padding and its final register and memory state is compared with the
instruction-set reference model in rv32i.py.

The suite runs on two builds: BPRED=1 (branch target buffer) and BPRED=0
(static not-taken), which also gives the CPI comparison in the README.
"""
import json
import os
import random
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge

from programs import BENCHMARKS, random_case
from rv32i import *  # noqa: F401,F403

BASE = 0x2000                       # programs are placed and started here
BPRED = int(os.environ.get("BPRED", "1"))
RESULTS_DIR = Path(os.environ.get("RESULTS_DIR", "."))


# -----------------------------------------------------------------------------
# Harness
# -----------------------------------------------------------------------------
_clock_task = None
_dirty = {"code": 0, "data": 0}


def _ram(dut):
    return dut.u_soc.u_ram.mem


async def run(dut, program, dmem_init=None, max_cycles=400_000, cold=False):
    """Load a program at BASE, run it to EBREAK, return (regs, cycles, instret).

    The branch target buffer is a hint and survives reset, like it does on the
    board. `cold` clears it first, for measurements that must not depend on
    what ran before.
    """
    global _clock_task
    if _clock_task is None or _clock_task.done():   # cocotb kills it between tests
        _clock_task = cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())

    core = dut.u_soc.u_core
    dut.rst.value = 1
    dut.uart_rx.value = 1
    dut.gpio_in.value = 0
    await ClockCycles(dut.clk, 2)           # reset takes effect before memory is rewritten

    # The register file is a block RAM and has no reset: clear it by hand, then
    # clear only the memory the previous program could have touched.
    for r in range(32):
        core.u_regfile.mem[r].value = 0
    if cold and BPRED:
        for i in range(256):
            core.g_btb.u_btb.mem[i].value = 0
    ram = _ram(dut)
    for i in range(_dirty["code"]):
        ram[(BASE >> 2) + i].value = 0
    for i in range(_dirty["data"]):
        ram[i].value = 0
    for i, w in enumerate(program):
        ram[(BASE >> 2) + i].value = w
    for addr, w in (dmem_init or {}).items():
        ram[addr >> 2].value = w & MASK
    _dirty["code"] = len(program) + 8
    _dirty["data"] = 0x500 // 4

    await ClockCycles(dut.clk, 3)
    dut.rst.value = 0

    # The reset sweep of the cache tags comes first; count from the first fetch.
    start = None
    for _ in range(max_cycles):
        await RisingEdge(dut.clk)
        if start is None and core.ic_busy.value == 0:
            start = core.u_csr.mcycle_q.value.to_unsigned()
        if core.m_trap.value == 1:
            break
    else:
        raise AssertionError("processor did not reach EBREAK")

    cause = core.m_trap_cause.value.to_unsigned()
    assert cause == 3, \
        f"unexpected trap, cause {cause} at pc 0x{core.m_pc.value.to_unsigned() << 2:x}"
    cycles = core.u_csr.mcycle_q.value.to_unsigned() - start
    instret = core.u_csr.minstret_q.value.to_unsigned()

    # The instruction in W writes on this edge; everything younger than the
    # EBREAK is squashed. Stop the core before the trap handler can run.
    dut.rst.value = 1
    await ClockCycles(dut.clk, 2)
    regs = [0] + [core.u_regfile.mem[r].value.to_unsigned() for r in range(1, 32)]
    return regs, cycles, instret


def dmem_word(dut, addr):
    return _ram(dut)[addr >> 2].value.to_unsigned()


async def check_against_model(dut, program, dmem_init=None, dmem_check=256, tag="", cold=False):
    ref = ISS(program, base=BASE, dmem_init=dmem_init)
    ref.run()
    regs, cycles, instret = await run(dut, program, dmem_init, cold=cold)
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
    """Back-to-back RAW chain."""
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
        ADDI(14, 0, 5),         # register-file bypasses: read 3 and 4 cycles
        NOP(), NOP(),           # after the write (W and post-W)
        ADD(15, 14, 14),
        ADDI(16, 0, 6),
        NOP(), NOP(), NOP(),
        ADD(17, 16, 16),
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
        JAL(1, 12),             # +0x00 -> +0x0C, x1 = +0x04
        ADDI(5, 0, 1),          # +0x04 skipped... returned to later
        EBREAK(),               # +0x08
        ADDI(6, 0, 2),          # +0x0C
        JALR(0, 1, 0),          # +0x10 return to +0x04 (x1 forwarded from MEM/WB)
    ]
    await check_against_model(dut, prog, tag="jal")

    prog = [
        AUIPC(3, 0),            # +0x00 x3 = pc
        ADDI(3, 3, 0x11),       # +0x04 x3 = pc + 0x11 (bit 0 must be cleared)
        JALR(4, 3, 4),          # +0x08 -> +0x14, x4 = +0x0C
        ADDI(7, 0, 1),          # +0x0C skipped
        ADDI(8, 0, 1),          # +0x10 skipped
        ADDI(9, 4, 0),          # +0x14 x9 = link
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


@cocotb.test()
async def test_muldiv(dut):
    """Every M-extension operation on the corner cases the ISA calls out."""
    values = [0, 1, 2, 7, 0x7FFFFFFF, 0x80000000, 0xFFFFFFFF, 0xFFFFFFFE,
              0x12345678, 0xDEADBEEF, 0x0000FFFF, 0x00010000]
    rng = random.Random(7)
    pairs = [(a, b) for a in values for b in values]
    rng.shuffle(pairs)
    for chunk in range(0, len(pairs), 8):
        prog, ptr = [], 0
        for a, b in pairs[chunk:chunk + 8]:
            prog += [*li(1, a), *li(2, b)]
            for fn in MULDIV:
                prog += [fn(3, 1, 2), SW(3, 0, ptr)]      # result consumed at once
                ptr += 4
        prog += [EBREAK()]
        await check_against_model(dut, prog, dmem_check=ptr // 4, tag=f"muldiv{chunk}")


@cocotb.test()
async def test_divide_in_loop(dut):
    """Decimal conversion as GCC compiles it: REMU and DIVU around a predicted
    loop branch, with the quotient consumed by the first instruction of the
    next iteration."""
    for value in (80000, 333833500, 7, 0xFFFFFFFF, 1000000000, 10):
        prog = [
            *li(10, value), ADDI(11, 0, 10), ADDI(29, 0, 87), ADDI(16, 0, 0),
            ADDI(6, 0, 0x100), ADDI(30, 0, 9),
            JAL(0, 8),
            ADDI(10, 15, 0),            # loop: a0 = quotient
            REMU(15, 10, 11),
            ADDI(16, 16, 1),
            ADD(28, 6, 16),
            ADD(17, 15, 29),
            BLTU(30, 15, 8),
            ADDI(17, 15, 48),
            SB(17, 28, -1),
            DIVU(15, 10, 11),
            BGEU(10, 11, -36),
            EBREAK(),
        ]
        await check_against_model(dut, prog, dmem_check=0x140 // 4, tag=f"itoa({value})")


@cocotb.test()
async def test_stale_prediction_on_mmio_store(dut):
    """A stale "taken" prediction on a store to a peripheral must cost a
    refetch, never the store. The first program trains the branch target
    buffer on a branch; the second has an MMIO store at the same address."""
    train = [ADDI(1, 0, 6), NOP(), NOP(), NOP(),
             ADDI(1, 1, -1),            # +0x10
             BNE(1, 0, -4),             # +0x14: taken five times
             EBREAK()]
    await run(dut, train, cold=True)
    store = [*li(5, 0x80002000),        # GPIO output register
             ADDI(6, 0, 0x55), NOP(), NOP(),
             SW(6, 5, 0),               # +0x14: predicted taken by the stale entry
             ADDI(7, 0, 1),
             EBREAK()]
    seen = set()

    async def watch():                  # the harness resets the SoC when the program ends
        while True:
            await RisingEdge(dut.clk)
            seen.add(int(dut.gpio_out.value))

    watcher = cocotb.start_soon(watch())
    regs, _, _ = await run(dut, store)
    watcher.cancel()
    assert 0x55 in seen, "the MMIO store was squashed by its own misprediction"
    assert regs[7] == 1


@cocotb.test()
async def test_self_modifying_code(dut):
    """A store into code that is already cached takes effect after FENCE.I."""
    patch = ADDI(5, 0, 222)
    prog = [
        JAL(0, 12),                 # +0x00 -> +0x0C
        ADDI(5, 0, 111),            # +0x04 to be patched
        JALR(0, 1, 0),              # +0x08 return
        JAL(1, -8),                 # +0x0C call +0x04: x5 = 111
        ADDI(6, 5, 0),              # +0x10
        AUIPC(7, 0),                # +0x14
        *li(8, patch),              # +0x18, +0x1C
        SW(8, 7, -0x10),            # +0x20 overwrite +0x04
        FENCE_I(),                  # +0x24
        JAL(1, -0x24),              # +0x28 call +0x04 again: x5 = 222
        EBREAK(),                   # +0x2C
    ]
    regs, _, _ = await run(dut, prog)
    assert regs[6] == 111 and regs[5] == 222, (regs[6], regs[5])


# -----------------------------------------------------------------------------
# Cycle-exact timing
# -----------------------------------------------------------------------------
@cocotb.test()
async def test_timing(dut):
    """Hazard penalties, measured in cycles on otherwise identical programs."""
    base = [ADDI(1, 0, 1), ADDI(2, 0, 2), ADD(3, 1, 1), ADD(4, 2, 2), EBREAK()]
    dep  = [ADDI(1, 0, 1), ADDI(2, 1, 2), ADD(3, 2, 1), ADD(4, 3, 2), EBREAK()]
    _, c_base, _ = await run(dut, base, cold=True)
    _, c_dep, _ = await run(dut, dep, cold=True)
    assert c_dep == c_base, f"dependent ALU chain stalled: {c_dep} vs {c_base}"

    lu_free = [ADDI(1, 0, 4), LW(2, 0, 0), ADDI(3, 1, 1), ADDI(4, 2, 1), EBREAK()]
    lu      = [ADDI(1, 0, 4), LW(2, 0, 0), ADDI(3, 2, 1), ADDI(4, 1, 1), EBREAK()]
    _, c_free, _ = await run(dut, lu_free, cold=True)
    _, c_lu, _ = await run(dut, lu, cold=True)
    assert c_lu == c_free + 1, f"load-use penalty {c_lu - c_free}, expected 1"

    mu_free = [ADDI(1, 0, 4), MUL(2, 1, 1), ADDI(3, 1, 1), ADDI(4, 2, 1), EBREAK()]
    mu      = [ADDI(1, 0, 4), MUL(2, 1, 1), ADDI(3, 2, 1), ADDI(4, 1, 1), EBREAK()]
    _, c_free, _ = await run(dut, mu_free, cold=True)
    _, c_mu, _ = await run(dut, mu, cold=True)
    assert c_mu == c_free + 1, f"MUL-use penalty {c_mu - c_free}, expected 1"

    sl_free = [ADDI(1, 0, 4), SW(1, 0, 0), ADDI(3, 1, 1), LW(2, 0, 0), EBREAK()]
    sl      = [ADDI(1, 0, 4), ADDI(3, 1, 1), SW(1, 0, 0), LW(2, 0, 0), EBREAK()]
    _, c_free, _ = await run(dut, sl_free, cold=True)
    _, c_sl, _ = await run(dut, sl, cold=True)
    assert c_sl == c_free + 1, f"store-then-load penalty {c_sl - c_free}, expected 1"

    nt = [ADDI(1, 0, 1), BEQ(1, 0, 8), ADDI(2, 0, 1), EBREAK()]   # not taken
    tk = [ADDI(1, 0, 1), BNE(1, 0, 4), ADDI(2, 0, 1), EBREAK()]   # taken to next
    _, c_nt, _ = await run(dut, nt, cold=True)
    _, c_tk, _ = await run(dut, tk, cold=True)
    assert c_tk == c_nt + 3, f"unpredicted taken-branch penalty {c_tk - c_nt}, expected 3"

    # A loop branch: the first taken iteration is a miss, every later one is
    # predicted and costs nothing, the exit is a misprediction.
    def loop(n):
        return [ADDI(1, 0, n), ADDI(1, 1, -1), BNE(1, 0, -4), EBREAK()]
    _, c10, _ = await run(dut, loop(10), cold=True)
    _, c20, _ = await run(dut, loop(20), cold=True)
    per_iter = (c20 - c10) / 10
    expected = 2 if BPRED else 5
    assert per_iter == expected, f"{per_iter} cycles per loop iteration, expected {expected}"
    dut._log.info(f"dependent ALU +0, load-use +1, MUL-use +1, store-then-load +1, mispredict +3, "
                  f"loop iteration {per_iter:.0f} cycles for 2 instructions")


# -----------------------------------------------------------------------------
# Constrained-random programs vs. reference model
# -----------------------------------------------------------------------------
@cocotb.test()
async def test_random_programs(dut):
    seed = int(os.environ.get("SEED", "2026"))
    count = int(os.environ.get("RANDOM_PROGRAMS", "1000"))
    rng = random.Random(seed)
    total = 0
    for n in range(count):
        prog, init, ref = random_case(rng, BASE)
        await check_against_model(dut, prog, dmem_init=init, dmem_check=0x200 // 4,
                                  tag=f"seed={seed} prog={n}")
        total += ref.retired
    dut._log.info(f"{count} random programs ({total} instructions executed) "
                  f"matched the reference model")


# -----------------------------------------------------------------------------
# Benchmarks: correctness + CPI
# -----------------------------------------------------------------------------
@cocotb.test()
async def test_benchmarks(dut):
    results = {}
    for name, gen in BENCHMARKS.items():
        prog = gen()
        cycles, instret = await check_against_model(dut, prog, dmem_check=0x500 // 4, tag=name,
                                                    cold=True)
        results[name] = {"cycles": cycles, "instret": instret, "cpi": round(cycles / instret, 3)}
        dut._log.info(f"{name:16s} cycles={cycles:6d} instret={instret:6d} "
                      f"CPI={cycles / instret:.3f}")
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    (RESULTS_DIR / f"cpi_bpred{BPRED}.json").write_text(json.dumps(results, indent=2))
