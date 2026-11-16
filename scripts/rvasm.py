#!/usr/bin/env python3
"""A small RV32IM assembler and disassembler for the interactive console.

    words = assemble(["li a0, 10", "loop:", "addi a0, a0, -1", "bnez a0, loop"], base=0x2000)
    text  = disassemble(0x00A00513, pc=0x2000)

Supports the RV32IM instructions, the CSR instructions, labels, and the usual
pseudo-instructions (nop, li, mv, not, neg, j, jr, ret, call, beqz, bnez, bgt,
ble, bgtu, bleu, csrr, csrw). Branch and jump targets are a label, an absolute
address (0x...) or a relative offset written with a sign (+8, -4).
"""
import os
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "test"))
import rv32i as rv      # noqa: E402

ABI = ["zero", "ra", "sp", "gp", "tp", "t0", "t1", "t2", "s0", "s1", "a0", "a1", "a2", "a3",
       "a4", "a5", "a6", "a7", "s2", "s3", "s4", "s5", "s6", "s7", "s8", "s9", "s10", "s11",
       "t3", "t4", "t5", "t6"]
REGS = {name: i for i, name in enumerate(ABI)}
REGS.update({f"x{i}": i for i in range(32)})
REGS["fp"] = 8

CSRS = {"mstatus": 0x300, "misa": 0x301, "mie": 0x304, "mtvec": 0x305, "mscratch": 0x340,
        "mepc": 0x341, "mcause": 0x342, "mtval": 0x343, "mip": 0x344, "mcycle": 0xB00,
        "minstret": 0xB02, "mcycleh": 0xB80, "minstreth": 0xB82, "mhpmcounter3": 0xB03,
        "mhpmcounter4": 0xB04, "mhpmcounter5": 0xB05, "mhpmcounter6": 0xB06,
        "cycle": 0xC00, "time": 0xC01, "instret": 0xC02}
CSR_NAMES = {v: k for k, v in CSRS.items()}

R_OPS = {n.lower(): getattr(rv, n) for n in
         "ADD SUB SLL SLT SLTU XOR SRL SRA OR AND MUL MULH MULHSU MULHU DIV DIVU REM REMU".split()}
I_OPS = {n.lower(): getattr(rv, n) for n in "ADDI SLTI SLTIU XORI ORI ANDI".split()}
SH_OPS = {n.lower(): getattr(rv, n) for n in "SLLI SRLI SRAI".split()}
LOADS = {n.lower(): getattr(rv, n) for n in "LB LH LW LBU LHU".split()}
STORES = {n.lower(): getattr(rv, n) for n in "SB SH SW".split()}
BRANCHES = {n.lower(): getattr(rv, n) for n in "BEQ BNE BLT BGE BLTU BGEU".split()}
SWAPPED = {"bgt": "blt", "ble": "bge", "bgtu": "bltu", "bleu": "bgeu"}
CSR_OPS = {"csrrw": 1, "csrrs": 2, "csrrc": 3, "csrrwi": 5, "csrrsi": 6, "csrrci": 7}


class AsmError(ValueError):
    pass


def reg(text):
    try:
        return REGS[text.strip().lower()]
    except KeyError:
        raise AsmError(f"unknown register '{text.strip()}'") from None


def number(text):
    try:
        return int(text.strip().replace("_", ""), 0)
    except ValueError:
        raise AsmError(f"bad number '{text.strip()}'") from None


def csr(text):
    name = text.strip().lower()
    return CSRS[name] if name in CSRS else number(name)


def _mem(text):
    """'imm(reg)' -> (imm, reg)"""
    m = re.fullmatch(r"\s*(.*?)\s*\(\s*(\w+)\s*\)\s*", text)
    if not m:
        raise AsmError(f"expected offset(register), got '{text.strip()}'")
    return (number(m.group(1)) if m.group(1) else 0), reg(m.group(2))


def _check(value, bits, what):
    if not -(1 << (bits - 1)) <= value < (1 << (bits - 1)):
        raise AsmError(f"{what} {value} does not fit in {bits} bits")
    return value


def _csr_word(csr_addr, rs1, f3, rd):
    return ((csr_addr & 0xFFF) << 20) | (rs1 << 15) | (f3 << 12) | (rd << 7) | 0x73


def _split(line):
    line = line.split("#")[0].split("//")[0].strip()
    if not line:
        return None, []
    parts = line.split(None, 1)
    ops = [o.strip() for o in parts[1].split(",")] if len(parts) > 1 else []
    return parts[0].lower(), ops


def size_of(line):
    """Number of instruction words a source line assembles to."""
    op, ops = _split(line)
    if op is None or op.endswith(":"):
        return 0
    if op == "li" and len(ops) == 2:
        value = number(ops[1]) & rv.MASK
        signed = value - (1 << 32) if value >> 31 else value
        return 1 if -2048 <= signed < 2048 or value & 0xFFF == 0 else 2
    return 1


def _target(text, pc, labels):
    text = text.strip()
    if text in labels:
        return labels[text] - pc
    if text[:1] in "+-":
        return number(text)
    try:
        return number(text) - pc
    except AsmError:
        raise AsmError(f"unknown label '{text}'") from None


def encode(line, pc=0, labels=None):
    """Assemble one source line into a list of instruction words."""
    labels = labels or {}
    op, o = _split(line)
    if op is None or op.endswith(":"):
        return []

    def need(n):
        if len(o) != n:
            raise AsmError(f"'{op}' takes {n} operand(s), got {len(o)}")

    if op in R_OPS:
        need(3)
        return [R_OPS[op](reg(o[0]), reg(o[1]), reg(o[2]))]
    if op in I_OPS:
        need(3)
        return [I_OPS[op](reg(o[0]), reg(o[1]), _check(number(o[2]), 12, "immediate"))]
    if op in SH_OPS:
        need(3)
        amount = number(o[2])
        if not 0 <= amount < 32:
            raise AsmError("shift amount must be 0..31")
        return [SH_OPS[op](reg(o[0]), reg(o[1]), amount)]
    if op in LOADS:
        need(2)
        imm, base = _mem(o[1])
        return [LOADS[op](reg(o[0]), base, _check(imm, 12, "offset"))]
    if op in STORES:
        need(2)
        imm, base = _mem(o[1])
        return [STORES[op](reg(o[0]), base, _check(imm, 12, "offset"))]
    if op in BRANCHES or op in SWAPPED:
        need(3)
        a, b = reg(o[0]), reg(o[1])
        if op in SWAPPED:
            op, a, b = SWAPPED[op], b, a
        return [BRANCHES[op](a, b, _check(_target(o[2], pc, labels), 13, "branch offset"))]
    if op in ("beqz", "bnez"):
        need(2)
        fn = rv.BEQ if op == "beqz" else rv.BNE
        return [fn(reg(o[0]), 0, _check(_target(o[1], pc, labels), 13, "branch offset"))]
    if op in ("lui", "auipc"):
        need(2)
        value = number(o[1])
        if not 0 <= value < (1 << 20):
            raise AsmError("upper immediate must be 0..0xFFFFF")
        return [(rv.LUI if op == "lui" else rv.AUIPC)(reg(o[0]), value)]
    if op == "jal":
        if len(o) not in (1, 2):
            need(2)
        rd, target = (reg(o[0]), o[1]) if len(o) == 2 else (1, o[0])
        return [rv.JAL(rd, _check(_target(target, pc, labels), 21, "jump offset"))]
    if op in ("j", "call"):
        need(1)
        return [rv.JAL(1 if op == "call" else 0, _check(_target(o[0], pc, labels), 21, "jump offset"))]
    if op == "jalr":
        if len(o) == 1:
            return [rv.JALR(1, reg(o[0]), 0)]
        if len(o) == 2:
            imm, base = _mem(o[1])
            return [rv.JALR(reg(o[0]), base, _check(imm, 12, "offset"))]
        need(3)
        return [rv.JALR(reg(o[0]), reg(o[1]), _check(number(o[2]), 12, "offset"))]
    if op == "jr":
        need(1)
        return [rv.JALR(0, reg(o[0]), 0)]
    if op == "ret":
        need(0)
        return [rv.JALR(0, 1, 0)]
    if op == "nop":
        need(0)
        return [rv.NOP()]
    if op == "li":
        need(2)
        value = number(o[1]) & rv.MASK
        signed = value - (1 << 32) if value >> 31 else value
        if -2048 <= signed < 2048:
            return [rv.ADDI(reg(o[0]), 0, signed)]
        if value & 0xFFF == 0:
            return [rv.LUI(reg(o[0]), value >> 12)]
        return rv.li(reg(o[0]), value)
    if op == "mv":
        need(2)
        return [rv.ADDI(reg(o[0]), reg(o[1]), 0)]
    if op == "not":
        need(2)
        return [rv.XORI(reg(o[0]), reg(o[1]), -1)]
    if op == "neg":
        need(2)
        return [rv.SUB(reg(o[0]), 0, reg(o[1]))]
    if op in CSR_OPS:
        need(3)
        f3 = CSR_OPS[op]
        src = number(o[2]) & 31 if f3 >= 5 else reg(o[2])
        return [_csr_word(csr(o[1]), src, f3, reg(o[0]))]
    if op == "csrr":
        need(2)
        return [_csr_word(csr(o[1]), 0, 2, reg(o[0]))]
    if op == "csrw":
        need(2)
        return [_csr_word(csr(o[0]), reg(o[1]), 1, 0)]
    if op in ("ecall", "ebreak", "mret", "wfi"):
        need(0)
        return [{"ecall": 0x00000073, "ebreak": 0x00100073, "mret": 0x30200073, "wfi": 0x10500073}[op]]
    if op == "fence.i":
        return [rv.FENCE_I()]
    if op == ".word":
        need(1)
        return [number(o[0]) & rv.MASK]
    raise AsmError(f"unknown instruction '{op}'")


def is_label(line):
    op, ops = _split(line)
    return op is not None and op.endswith(":") and not ops


def assemble(lines, base=0):
    """Two-pass assembly of source lines; returns the instruction words."""
    labels, pc = {}, base
    for line in lines:
        if is_label(line):
            labels[_split(line)[0][:-1]] = pc
        pc += 4 * size_of(line)
    words, pc = [], base
    for line in lines:
        out = encode(line, pc, labels)
        words += out
        pc += 4 * len(out)
    return words


# -----------------------------------------------------------------------------
# Disassembler
# -----------------------------------------------------------------------------
def _sx(value, bits):
    value &= (1 << bits) - 1
    return value - (1 << bits) if value >> (bits - 1) else value


def fields(word):
    """Decoded fields of an instruction word."""
    return {
        "op": word & 0x7F, "rd": (word >> 7) & 31, "f3": (word >> 12) & 7,
        "rs1": (word >> 15) & 31, "rs2": (word >> 20) & 31, "f7": word >> 25,
        "imm_i": _sx(word >> 20, 12),
        "imm_s": _sx(((word >> 25) << 5) | ((word >> 7) & 31), 12),
        "imm_b": _sx((((word >> 31) & 1) << 12) | (((word >> 7) & 1) << 11) |
                     (((word >> 25) & 0x3F) << 5) | (((word >> 8) & 0xF) << 1), 13),
        "imm_j": _sx((((word >> 31) & 1) << 20) | (((word >> 12) & 0xFF) << 12) |
                     (((word >> 20) & 1) << 11) | (((word >> 21) & 0x3FF) << 1), 21),
    }


def disassemble(word, pc=None):
    f = fields(word)
    op, rd, f3, rs1, rs2, f7 = (f[k] for k in ("op", "rd", "f3", "rs1", "rs2", "f7"))
    r = ABI

    def target(offset):
        return f"{offset:+d}" if pc is None else f"0x{(pc + offset) & rv.MASK:x}"

    if word == 0x00000013:
        return "nop"
    if op == 0b0110111:
        return f"lui {r[rd]}, 0x{word >> 12:x}"
    if op == 0b0010111:
        return f"auipc {r[rd]}, 0x{word >> 12:x}"
    if op == 0b1101111:
        return f"j {target(f['imm_j'])}" if rd == 0 else f"jal {r[rd]}, {target(f['imm_j'])}"
    if op == 0b1100111 and f3 == 0:
        if rd == 0 and rs1 == 1 and f["imm_i"] == 0:
            return "ret"
        return f"jalr {r[rd]}, {f['imm_i']}({r[rs1]})"
    if op == 0b1100011 and f3 not in (2, 3):
        name = {0: "beq", 1: "bne", 4: "blt", 5: "bge", 6: "bltu", 7: "bgeu"}[f3]
        return f"{name} {r[rs1]}, {r[rs2]}, {target(f['imm_b'])}"
    if op == 0b0000011 and f3 in (0, 1, 2, 4, 5):
        name = {0: "lb", 1: "lh", 2: "lw", 4: "lbu", 5: "lhu"}[f3]
        return f"{name} {r[rd]}, {f['imm_i']}({r[rs1]})"
    if op == 0b0100011 and f3 in (0, 1, 2):
        return f"{('sb', 'sh', 'sw')[f3]} {r[rs2]}, {f['imm_s']}({r[rs1]})"
    if op == 0b0010011:
        if f3 == 1 and f7 == 0:
            return f"slli {r[rd]}, {r[rs1]}, {rs2}"
        if f3 == 5 and f7 in (0, 0x20):
            return f"{'srai' if f7 else 'srli'} {r[rd]}, {r[rs1]}, {rs2}"
        if f3 not in (1, 5):
            if f3 == 0 and rs1 == 0:
                return f"li {r[rd]}, {f['imm_i']}"
            if f3 == 0 and f["imm_i"] == 0:
                return f"mv {r[rd]}, {r[rs1]}"
            name = {0: "addi", 2: "slti", 3: "sltiu", 4: "xori", 6: "ori", 7: "andi"}[f3]
            return f"{name} {r[rd]}, {r[rs1]}, {f['imm_i']}"
    if op == 0b0110011:
        names = None
        if f7 == 0:
            names = ["add", "sll", "slt", "sltu", "xor", "srl", "or", "and"]
        elif f7 == 0x20 and f3 in (0, 5):
            names = ["sub", "", "", "", "", "sra", "", ""]
        elif f7 == 1:
            names = ["mul", "mulh", "mulhsu", "mulhu", "div", "divu", "rem", "remu"]
        if names:
            return f"{names[f3]} {r[rd]}, {r[rs1]}, {r[rs2]}"
    if op == 0b0001111 and f3 in (0, 1):
        return "fence.i" if f3 else "fence"
    if op == 0b1110011:
        fixed = {0x00000073: "ecall", 0x00100073: "ebreak", 0x30200073: "mret", 0x10500073: "wfi"}
        if word in fixed:
            return fixed[word]
        if f3 in (1, 2, 3, 5, 6, 7):
            name = CSR_NAMES.get(word >> 20, f"0x{word >> 20:x}")
            if f3 == 2 and rs1 == 0:
                return f"csrr {r[rd]}, {name}"
            if f3 == 1 and rd == 0:
                return f"csrw {name}, {r[rs1]}"
            op_name = {1: "csrrw", 2: "csrrs", 3: "csrrc", 5: "csrrwi", 6: "csrrsi", 7: "csrrci"}[f3]
            return f"{op_name} {r[rd]}, {name}, {rs1 if f3 >= 5 else r[rs1]}"
    return f".word 0x{word:08x}"


if __name__ == "__main__":
    for text in sys.argv[1:]:
        for w in encode(text):
            print(f"{w:08x}  {disassemble(w)}")
