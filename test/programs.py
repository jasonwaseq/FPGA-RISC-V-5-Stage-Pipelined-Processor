"""Test programs shared by the simulation and the hardware test suites:
benchmark kernels for CPI measurement (hand-assembled with labels) and the
constrained-random program generator.

Each entry in a program list is either a label string ending in ':' or an
instruction word, or a callable f(pc, labels) -> word for PC-relative
instructions that reference a label.
"""
from rv32i import *  # noqa: F401,F403


def assemble(items):
    labels, pc = {}, 0
    for it in items:
        if isinstance(it, str):
            labels[it.rstrip(":")] = pc
        else:
            pc += 4
    words, pc = [], 0
    for it in items:
        if isinstance(it, str):
            continue
        words.append(it(pc, labels) if callable(it) else it)
        pc += 4
    return words


def br(fn, a, b, label):
    return lambda pc, L: fn(a, b, L[label] - pc)


def jal(rd, label):
    return lambda pc, L: JAL(rd, L[label] - pc)


def fibonacci(n=24):
    """fib[i] = fib[i-1] + fib[i-2], stored to dmem[0..n)."""
    return assemble([
        ADDI(1, 0, 0),          # a = 0
        ADDI(2, 0, 1),          # b = 1
        ADDI(3, 0, 0),          # ptr
        ADDI(4, 0, n),          # count
        "loop:",
        SW(1, 3, 0),
        ADD(5, 1, 2),           # t = a + b
        ADDI(1, 2, 0),          # a = b
        ADDI(2, 5, 0),          # b = t
        ADDI(3, 3, 4),
        ADDI(4, 4, -1),
        br(BNE, 4, 0, "loop"),
        EBREAK(),
    ])


def array_sum(n=64):
    """Fill a[i] = 3*i + 1, then sum it back with loads (load-use heavy)."""
    return assemble([
        ADDI(1, 0, 0),          # i*4
        ADDI(2, 0, 1),          # value
        ADDI(3, 0, n * 4),
        "fill:",
        SW(2, 1, 0),
        ADDI(2, 2, 3),
        ADDI(1, 1, 4),
        br(BNE, 1, 3, "fill"),
        ADDI(1, 0, 0),
        ADDI(10, 0, 0),         # sum
        "sum:",
        LW(4, 1, 0),
        ADD(10, 10, 4),         # load-use
        ADDI(1, 1, 4),
        br(BNE, 1, 3, "sum"),
        SW(10, 0, 0x400),
        EBREAK(),
    ])


def bubble_sort(n=16):
    """Fill a descending array and bubble-sort it in place."""
    return assemble([
        ADDI(1, 0, 0),
        ADDI(2, 0, n),
        "init:",
        SW(2, 1, 0),
        ADDI(1, 1, 4),
        ADDI(2, 2, -1),
        br(BNE, 2, 0, "init"),
        ADDI(5, 0, n - 1),      # outer passes
        "outer:",
        ADDI(1, 0, 0),          # ptr
        ADDI(6, 0, 0),          # j
        "inner:",
        LW(3, 1, 0),
        LW(4, 1, 4),
        br(BGE, 4, 3, "noswap"),
        SW(4, 1, 0),
        SW(3, 1, 4),
        "noswap:",
        ADDI(1, 1, 4),
        ADDI(6, 6, 1),
        br(BLT, 6, 5, "inner"),
        ADDI(5, 5, -1),
        br(BNE, 5, 0, "outer"),
        EBREAK(),
    ])


def dependent_chain(n=24, passes=8):
    """A loop whose body is a chain: every instruction consumes the previous result."""
    body = [[ADD(1, 1, 1), XORI(1, 1, 0x55), SUB(1, 1, 0), ORI(1, 1, 3)][i % 4] for i in range(n)]
    return assemble([
        ADDI(1, 0, 1),
        ADDI(2, 0, passes),
        "loop:",
        *body,
        ADDI(2, 2, -1),
        br(BNE, 2, 0, "loop"),
        EBREAK(),
    ])


def matmul(n=6):
    """C = A * B on n x n matrices of small integers (MUL and load-use heavy)."""
    a, b, c = 0x000, 0x100, 0x200
    return assemble([
        ADDI(1, 0, 0),                  # fill A[i] = i + 1, B[i] = 2i + 3
        ADDI(2, 0, n * n * 4),
        ADDI(3, 0, 1),
        ADDI(4, 0, 3),
        "fill:",
        SW(3, 1, a),
        SW(4, 1, b),
        ADDI(3, 3, 1),
        ADDI(4, 4, 2),
        ADDI(1, 1, 4),
        br(BNE, 1, 2, "fill"),
        ADDI(5, 0, 0),                  # i * n * 4
        ADDI(20, 0, n * 4),
        "row:",
        ADDI(6, 0, 0),                  # j * 4
        "col:",
        ADDI(7, 0, 0),                  # k * 4
        ADDI(8, 0, 0),                  # k * n * 4
        ADDI(10, 0, 0),                 # sum
        "dot:",
        ADD(11, 5, 7),
        LW(12, 11, a),                  # A[i][k]
        ADD(13, 8, 6),
        LW(14, 13, b),                  # B[k][j]
        MUL(15, 12, 14),
        ADD(10, 10, 15),
        ADDI(7, 7, 4),
        ADDI(8, 8, n * 4),
        br(BNE, 7, 20, "dot"),
        ADD(11, 5, 6),
        SW(10, 11, c),
        ADDI(6, 6, 4),
        br(BNE, 6, 20, "col"),
        ADDI(5, 5, n * 4),
        br(BNE, 5, 2, "row"),
        EBREAK(),
    ])


BENCHMARKS = {
    "fibonacci": fibonacci,
    "array_sum": array_sum,
    "bubble_sort": bubble_sort,
    "dependent_chain": dependent_chain,
    "matmul": matmul,
}


# -----------------------------------------------------------------------------
# Constrained-random programs
# -----------------------------------------------------------------------------
def random_program(rng, length):
    """Random RV32IM code with dense hazards.

    Few registers, so almost every instruction depends on a recent one. Control
    flow is forward branches and jumps plus counted loops (x10), so the branch
    target buffer sees taken, not-taken and mispredicted branches.
    """
    regs = list(range(1, 8))
    prog = [*li(9, 0x100), ADDI(1, 0, rng.randint(-2048, 2047))]
    loop_start = None
    while len(prog) < length:
        k = rng.random()
        rd, a, b = rng.choice(regs), rng.choice(regs + [0]), rng.choice(regs + [0])
        remaining = length - len(prog)
        if k < 0.26:
            prog.append(rng.choice(ALU_RR)(rd, a, b))
        elif k < 0.44:
            prog.append(rng.choice(ALU_RI)(rd, a, rng.randint(-2048, 2047)))
        elif k < 0.49:
            prog.append(rng.choice(SHIFT_I)(rd, a, rng.randint(0, 31)))
        elif k < 0.54:
            prog.append(rng.choice([LUI, AUIPC])(rd, rng.randint(0, 0xFFFFF)))
        elif k < 0.62:
            prog.append(rng.choice(MULDIV)(rd, a, b))
        elif k < 0.76:                      # load/store relative to x9 (fixed base)
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
        elif k < 0.90 and remaining > 3:    # forward branch
            prog.append(rng.choice(BRANCHES)(a, b, 4 * rng.randint(1, min(4, remaining - 1))))
        elif k < 0.93 and remaining > 3:
            prog.append(JAL(rng.choice(regs + [0]), 4 * rng.randint(1, min(4, remaining - 1))))
        elif k < 0.96 and remaining > 4:    # JALR to a forward target
            skip = rng.randint(0, min(3, remaining - 3))
            prog += [AUIPC(8, 0), JALR(rng.choice(regs + [0]), 8, 8 + 4 * skip)]
        elif loop_start is None and remaining > 12:     # open a counted loop
            prog.append(ADDI(10, 0, rng.randint(2, 6)))
            loop_start = len(prog)
        elif loop_start is not None and len(prog) - loop_start >= 3:   # close it
            prog += [ADDI(10, 10, -1), BNE(10, 0, 4 * (loop_start - len(prog) - 1))]
            loop_start = None
    return prog[:length] + [EBREAK()]


def random_case(rng, base=0):
    """A random program the reference model can finish, plus its data image.

    A branch can skip the instruction that closes a loop or feeds a JALR, so
    some generated programs never terminate; those are thrown away.
    """
    while True:
        prog = random_program(rng, rng.randint(20, 80))
        init = {0x100 + 4 * i: rng.getrandbits(32) for i in range(40)}
        try:
            ref = ISS(prog, base=base, dmem_init=init)
            ref.run(max_steps=3000)
            return prog, init, ref
        except (RuntimeError, IndexError, KeyError, ValueError):
            continue
