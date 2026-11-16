"""RV32IM instruction encoders and an instruction-set reference model.

The reference model is the golden model for the random tests: it executes a
program one instruction at a time with no notion of a pipeline, caches or
branch prediction, so any difference from the RTL is a hazard, forwarding,
decode or memory-system bug.
"""

MASK = 0xFFFF_FFFF


# -----------------------------------------------------------------------------
# Encoders
# -----------------------------------------------------------------------------
def _r(f7, rs2, rs1, f3, rd, op):
    return (f7 << 25) | (rs2 << 20) | (rs1 << 15) | (f3 << 12) | (rd << 7) | op


def _i(imm, rs1, f3, rd, op):
    return ((imm & 0xFFF) << 20) | (rs1 << 15) | (f3 << 12) | (rd << 7) | op


def _s(imm, rs2, rs1, f3, op=0b0100011):
    imm &= 0xFFF
    return ((imm >> 5) << 25) | (rs2 << 20) | (rs1 << 15) | (f3 << 12) | ((imm & 0x1F) << 7) | op


def _b(imm, rs2, rs1, f3, op=0b1100011):
    imm &= 0x1FFF
    return (((imm >> 12) & 1) << 31) | (((imm >> 5) & 0x3F) << 25) | (rs2 << 20) | \
           (rs1 << 15) | (f3 << 12) | (((imm >> 1) & 0xF) << 8) | (((imm >> 11) & 1) << 7) | op


def _u(imm20, rd, op):
    return ((imm20 & 0xFFFFF) << 12) | (rd << 7) | op


def _j(imm, rd, op=0b1101111):
    imm &= 0x1FFFFF
    return (((imm >> 20) & 1) << 31) | (((imm >> 1) & 0x3FF) << 21) | (((imm >> 11) & 1) << 20) | \
           (((imm >> 12) & 0xFF) << 12) | (rd << 7) | op


OPI, OPR = 0b0010011, 0b0110011

def ADDI(rd, rs1, imm):  return _i(imm, rs1, 0, rd, OPI)
def SLTI(rd, rs1, imm):  return _i(imm, rs1, 2, rd, OPI)
def SLTIU(rd, rs1, imm): return _i(imm, rs1, 3, rd, OPI)
def XORI(rd, rs1, imm):  return _i(imm, rs1, 4, rd, OPI)
def ORI(rd, rs1, imm):   return _i(imm, rs1, 6, rd, OPI)
def ANDI(rd, rs1, imm):  return _i(imm, rs1, 7, rd, OPI)
def SLLI(rd, rs1, sh):   return _i(sh & 31, rs1, 1, rd, OPI)
def SRLI(rd, rs1, sh):   return _i(sh & 31, rs1, 5, rd, OPI)
def SRAI(rd, rs1, sh):   return _i(0x400 | (sh & 31), rs1, 5, rd, OPI)

def ADD(rd, a, b):  return _r(0x00, b, a, 0, rd, OPR)
def SUB(rd, a, b):  return _r(0x20, b, a, 0, rd, OPR)
def SLL(rd, a, b):  return _r(0x00, b, a, 1, rd, OPR)
def SLT(rd, a, b):  return _r(0x00, b, a, 2, rd, OPR)
def SLTU(rd, a, b): return _r(0x00, b, a, 3, rd, OPR)
def XOR(rd, a, b):  return _r(0x00, b, a, 4, rd, OPR)
def SRL(rd, a, b):  return _r(0x00, b, a, 5, rd, OPR)
def SRA(rd, a, b):  return _r(0x20, b, a, 5, rd, OPR)
def OR(rd, a, b):   return _r(0x00, b, a, 6, rd, OPR)
def AND(rd, a, b):  return _r(0x00, b, a, 7, rd, OPR)

def MUL(rd, a, b):    return _r(0x01, b, a, 0, rd, OPR)
def MULH(rd, a, b):   return _r(0x01, b, a, 1, rd, OPR)
def MULHSU(rd, a, b): return _r(0x01, b, a, 2, rd, OPR)
def MULHU(rd, a, b):  return _r(0x01, b, a, 3, rd, OPR)
def DIV(rd, a, b):    return _r(0x01, b, a, 4, rd, OPR)
def DIVU(rd, a, b):   return _r(0x01, b, a, 5, rd, OPR)
def REM(rd, a, b):    return _r(0x01, b, a, 6, rd, OPR)
def REMU(rd, a, b):   return _r(0x01, b, a, 7, rd, OPR)

def LB(rd, rs1, imm):  return _i(imm, rs1, 0, rd, 0b0000011)
def LH(rd, rs1, imm):  return _i(imm, rs1, 1, rd, 0b0000011)
def LW(rd, rs1, imm):  return _i(imm, rs1, 2, rd, 0b0000011)
def LBU(rd, rs1, imm): return _i(imm, rs1, 4, rd, 0b0000011)
def LHU(rd, rs1, imm): return _i(imm, rs1, 5, rd, 0b0000011)
def SB(rs2, rs1, imm): return _s(imm, rs2, rs1, 0)
def SH(rs2, rs1, imm): return _s(imm, rs2, rs1, 1)
def SW(rs2, rs1, imm): return _s(imm, rs2, rs1, 2)

def BEQ(a, b, off):  return _b(off, b, a, 0)
def BNE(a, b, off):  return _b(off, b, a, 1)
def BLT(a, b, off):  return _b(off, b, a, 4)
def BGE(a, b, off):  return _b(off, b, a, 5)
def BLTU(a, b, off): return _b(off, b, a, 6)
def BGEU(a, b, off): return _b(off, b, a, 7)

def LUI(rd, imm20):   return _u(imm20, rd, 0b0110111)
def AUIPC(rd, imm20): return _u(imm20, rd, 0b0010111)
def JAL(rd, off):     return _j(off, rd)
def JALR(rd, rs1, imm): return _i(imm, rs1, 0, rd, 0b1100111)

def NOP():    return ADDI(0, 0, 0)
def EBREAK(): return 0x0010_0073

def FENCE_I(): return 0x0000_100F

ALU_RR = [ADD, SUB, SLL, SLT, SLTU, XOR, SRL, SRA, OR, AND]
MULDIV = [MUL, MULH, MULHSU, MULHU, DIV, DIVU, REM, REMU]
ALU_RI = [ADDI, SLTI, SLTIU, XORI, ORI, ANDI]
SHIFT_I = [SLLI, SRLI, SRAI]
BRANCHES = [BEQ, BNE, BLT, BGE, BLTU, BGEU]


def li(rd, value):
    """Load a 32-bit constant (LUI + ADDI)."""
    value &= MASK
    lo = value & 0xFFF
    if lo & 0x800:
        lo -= 0x1000
    hi = ((value - lo) >> 12) & 0xFFFFF
    return [LUI(rd, hi), ADDI(rd, rd, lo)]


# -----------------------------------------------------------------------------
# Reference model
# -----------------------------------------------------------------------------
def _sx(v, bits):
    v &= (1 << bits) - 1
    return v - (1 << bits) if v >> (bits - 1) else v


def _s32(v):
    return _sx(v, 32)


def _muldiv(f3, a, b):
    """M extension on 32-bit unsigned register values."""
    sa, sb = _s32(a), _s32(b)
    if f3 == 0:
        return a * b
    if f3 == 1:
        return (sa * sb) >> 32
    if f3 == 2:
        return (sa * b) >> 32
    if f3 == 3:
        return (a * b) >> 32
    if f3 == 4:                                 # DIV: round towards zero
        if b == 0:
            return MASK
        if sa == -(1 << 31) and sb == -1:
            return a
        q = abs(sa) // abs(sb)
        return q if (sa < 0) == (sb < 0) else -q
    if f3 == 5:
        return MASK if b == 0 else a // b
    if f3 == 6:                                 # REM: sign of the dividend
        if b == 0:
            return a
        if sa == -(1 << 31) and sb == -1:
            return 0
        return (abs(sa) % abs(sb)) * (-1 if sa < 0 else 1)
    return a if b == 0 else a % b


class ISS:
    """Single-cycle RV32IM instruction-set simulator.

    One flat little-endian memory holds code and data. `program` is placed at
    `base`, which is also the reset PC. Execution stops at EBREAK / ECALL.
    """

    def __init__(self, program, base=0, mem_bytes=1 << 17, dmem_init=None):
        self.mem = bytearray(mem_bytes)
        self.base = base
        for i, word in enumerate(program):
            self.mem[base + 4 * i:base + 4 * i + 4] = (word & MASK).to_bytes(4, "little")
        if dmem_init:
            for addr, word in dmem_init.items():
                self.mem[addr:addr + 4] = (word & MASK).to_bytes(4, "little")
        self.x = [0] * 32
        self.pc = base
        self.retired = 0

    def load_word(self, addr):
        return int.from_bytes(self.mem[addr:addr + 4], "little")

    def _check(self, addr, size):
        """The core traps on these; the model refuses to run such a program."""
        if addr % size or addr + size > len(self.mem):
            raise ValueError(f"misaligned or out-of-range access at 0x{addr:08x}")

    def run(self, max_steps=100_000):
        for _ in range(max_steps):
            if self.step():
                return
        raise RuntimeError("reference model did not reach EBREAK")

    def step(self):
        self._check(self.pc, 4)
        ins = self.load_word(self.pc)
        op, rd = ins & 0x7F, (ins >> 7) & 31
        f3, rs1, rs2 = (ins >> 12) & 7, (ins >> 15) & 31, (ins >> 20) & 31
        f7 = ins >> 25
        a, b = self.x[rs1], self.x[rs2]
        imm_i = _sx(ins >> 20, 12)
        imm_s = _sx(((ins >> 25) << 5) | ((ins >> 7) & 31), 12)
        imm_b = _sx((((ins >> 31) & 1) << 12) | (((ins >> 7) & 1) << 11) |
                    (((ins >> 25) & 0x3F) << 5) | (((ins >> 8) & 0xF) << 1), 13)
        imm_u = ins & 0xFFFFF000
        imm_j = _sx((((ins >> 31) & 1) << 20) | (((ins >> 12) & 0xFF) << 12) |
                    (((ins >> 20) & 1) << 11) | (((ins >> 21) & 0x3FF) << 1), 21)
        next_pc = (self.pc + 4) & MASK
        res = None

        if op == 0b0110111:
            res = imm_u
        elif op == 0b0010111:
            res = (self.pc + imm_u) & MASK
        elif op == 0b1101111:
            res, next_pc = self.pc + 4, (self.pc + imm_j) & MASK
        elif op == 0b1100111:
            res, next_pc = self.pc + 4, (a + imm_i) & MASK & ~1
        elif op == 0b1100011:
            cond = {0: a == b, 1: a != b, 4: _s32(a) < _s32(b), 5: _s32(a) >= _s32(b),
                    6: a < b, 7: a >= b}.get(f3, False)
            if cond:
                next_pc = (self.pc + imm_b) & MASK
        elif op == 0b0000011:
            addr = (a + imm_i) & MASK
            self._check(addr, {0: 1, 1: 2, 2: 4, 4: 1, 5: 2}[f3])
            word = self.load_word(addr & ~3)
            sh = (addr & 3) * 8
            res = {0: _sx(word >> sh, 8) & MASK, 1: _sx(word >> sh, 16) & MASK,
                   2: word, 4: (word >> sh) & 0xFF, 5: (word >> sh) & 0xFFFF}[f3]
        elif op == 0b0100011:
            addr = (a + imm_s) & MASK
            n = {0: 1, 1: 2, 2: 4}[f3]
            self._check(addr, n)
            self.mem[addr:addr + n] = (b & ((1 << (8 * n)) - 1)).to_bytes(n, "little")
        elif op == 0b0110011 and f7 == 1:
            res = _muldiv(f3, a, b)
        elif op in (0b0010011, 0b0110011):
            rhs = imm_i & MASK if op == 0b0010011 else b
            sh = rhs & 31
            alt = (f7 >> 5) & 1
            if f3 == 0:
                res = a - rhs if (op == 0b0110011 and alt) else a + rhs
            elif f3 == 1:
                res = a << sh
            elif f3 == 2:
                res = int(_s32(a) < _s32(rhs))
            elif f3 == 3:
                res = int(a < (rhs & MASK))
            elif f3 == 4:
                res = a ^ rhs
            elif f3 == 5:
                res = (_s32(a) >> sh) if alt else (a >> sh)
            elif f3 == 6:
                res = a | rhs
            else:
                res = a & rhs
        elif op == 0b1110011 and f3 == 0:
            return True  # EBREAK / ECALL: traps, does not retire
        else:
            pass  # FENCE / FENCE.I: no architectural effect

        self.retired += 1
        if res is not None and rd != 0:
            self.x[rd] = res & MASK
        self.pc = next_pc
        return False
