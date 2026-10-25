"""Benchmark programs for CPI measurement (hand-assembled with labels).

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


def dependent_chain(n=200):
    """Straight-line code where every instruction consumes the previous result."""
    body = [ADDI(1, 0, 1)]
    for i in range(n):
        body.append([ADD(1, 1, 1), XORI(1, 1, 0x55), SUB(1, 1, 0), ORI(1, 1, 3)][i % 4])
    return assemble(body + [EBREAK()])


BENCHMARKS = {
    "fibonacci": fibonacci,
    "array_sum": array_sum,
    "bubble_sort": bubble_sort,
    "dependent_chain": dependent_chain,
}
