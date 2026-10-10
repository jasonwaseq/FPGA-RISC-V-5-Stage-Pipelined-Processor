#!/usr/bin/env python3
"""
RISC-V 5-Stage Pipelined Processor - FPGA Test & Interactive Suite
===================================================================

This script provides comprehensive testing and interactive control
of the RISC-V processor running on the iCEBreaker FPGA.

UART Protocol Commands:
  0xAA <addr_hi> <addr_lo> <d3> <d2> <d1> <d0> : Write instruction to memory
  0xBB <reg>                                   : Read register value (returns 4 bytes)
  0xCC                                         : Run processor
  0xDD                                         : Halt processor
  0xEE                                         : Reset processor (clears registers)
  0xBC <idx>                                   : Read counter (0 cycles, 1 instret, 2 halted)

The core implements the full RV32I base ISA (EBREAK/ECALL stop the core).
Hardware tests run programs with no NOP padding and compare every register
against the instruction-set reference model in test/rv32i.py.
"""

import serial
import serial.tools.list_ports
import time
import argparse
import os
import random
import sys
import re

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'test'))
import rv32i                      # noqa: E402  shared encoders + reference model
from programs import BENCHMARKS   # noqa: E402

# =============================================================================
# RV32I Instruction Encoding
# =============================================================================

# Register name to number mapping
REG_MAP = {
    'x0': 0, 'zero': 0,
    'x1': 1, 'ra': 1,
    'x2': 2, 'sp': 2,
    'x3': 3, 'gp': 3,
    'x4': 4, 'tp': 4,
    'x5': 5, 't0': 5,
    'x6': 6, 't1': 6,
    'x7': 7, 't2': 7,
    'x8': 8, 's0': 8, 'fp': 8,
    'x9': 9, 's1': 9,
    'x10': 10, 'a0': 10,
    'x11': 11, 'a1': 11,
    'x12': 12, 'a2': 12,
    'x13': 13, 'a3': 13,
    'x14': 14, 'a4': 14,
    'x15': 15, 'a5': 15,
    'x16': 16, 'a6': 16,
    'x17': 17, 'a7': 17,
    'x18': 18, 's2': 18,
    'x19': 19, 's3': 19,
    'x20': 20, 's4': 20,
    'x21': 21, 's5': 21,
    'x22': 22, 's6': 22,
    'x23': 23, 's7': 23,
    'x24': 24, 's8': 24,
    'x25': 25, 's9': 25,
    'x26': 26, 's10': 26,
    'x27': 27, 's11': 27,
    'x28': 28, 't3': 28,
    'x29': 29, 't4': 29,
    'x30': 30, 't5': 30,
    'x31': 31, 't6': 31,
}

def parse_reg(s):
    """Parse register name or number"""
    s = s.strip().lower()
    if s in REG_MAP:
        return REG_MAP[s]
    if s.startswith('x'):
        return int(s[1:])
    return int(s)

def parse_imm(s):
    """Parse immediate value (decimal or hex)"""
    s = s.strip()
    if s.startswith('0x') or s.startswith('-0x'):
        return int(s, 16)
    return int(s)

# Instruction encoding functions
def r_type(funct7, rs2, rs1, funct3, rd, opcode=0b0110011):
    return (funct7 << 25) | (rs2 << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | opcode

def i_type(imm, rs1, funct3, rd, opcode):
    return ((imm & 0xFFF) << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | opcode

def s_type(imm, rs2, rs1, funct3, opcode=0b0100011):
    imm_11_5 = (imm >> 5) & 0x7F
    imm_4_0 = imm & 0x1F
    return (imm_11_5 << 25) | (rs2 << 20) | (rs1 << 15) | (funct3 << 12) | (imm_4_0 << 7) | opcode

# I-type ALU instructions
def ADDI(rd, rs1, imm):  return i_type(imm, rs1, 0b000, rd, 0b0010011)
def SLTI(rd, rs1, imm):  return i_type(imm, rs1, 0b010, rd, 0b0010011)
def SLTIU(rd, rs1, imm): return i_type(imm, rs1, 0b011, rd, 0b0010011)
def XORI(rd, rs1, imm):  return i_type(imm, rs1, 0b100, rd, 0b0010011)
def ORI(rd, rs1, imm):   return i_type(imm, rs1, 0b110, rd, 0b0010011)
def ANDI(rd, rs1, imm):  return i_type(imm, rs1, 0b111, rd, 0b0010011)
def SLLI(rd, rs1, shamt): return i_type(shamt & 0x1F, rs1, 0b001, rd, 0b0010011)
def SRLI(rd, rs1, shamt): return i_type(shamt & 0x1F, rs1, 0b101, rd, 0b0010011)
def SRAI(rd, rs1, shamt): return i_type((0x400 | (shamt & 0x1F)), rs1, 0b101, rd, 0b0010011)

# R-type ALU instructions
def ADD(rd, rs1, rs2):  return r_type(0b0000000, rs2, rs1, 0b000, rd)
def SUB(rd, rs1, rs2):  return r_type(0b0100000, rs2, rs1, 0b000, rd)
def SLL(rd, rs1, rs2):  return r_type(0b0000000, rs2, rs1, 0b001, rd)
def SLT(rd, rs1, rs2):  return r_type(0b0000000, rs2, rs1, 0b010, rd)
def SLTU(rd, rs1, rs2): return r_type(0b0000000, rs2, rs1, 0b011, rd)
def XOR(rd, rs1, rs2):  return r_type(0b0000000, rs2, rs1, 0b100, rd)
def SRL(rd, rs1, rs2):  return r_type(0b0000000, rs2, rs1, 0b101, rd)
def SRA(rd, rs1, rs2):  return r_type(0b0100000, rs2, rs1, 0b101, rd)
def OR(rd, rs1, rs2):   return r_type(0b0000000, rs2, rs1, 0b110, rd)
def AND(rd, rs1, rs2):  return r_type(0b0000000, rs2, rs1, 0b111, rd)

# Load/Store instructions
def LW(rd, rs1, imm):   return i_type(imm, rs1, 0b010, rd, 0b0000011)
def SW(rs2, rs1, imm):  return s_type(imm, rs2, rs1, 0b010)

# Pseudo-instructions
def NOP():              return ADDI(0, 0, 0)
def LI(rd, imm):        return ADDI(rd, 0, imm)  # Load immediate (small values)
def MV(rd, rs1):        return ADDI(rd, rs1, 0)  # Move register

# Instruction assembler
INSTR_MAP = {
    'addi': lambda args: ADDI(parse_reg(args[0]), parse_reg(args[1]), parse_imm(args[2])),
    'slti': lambda args: SLTI(parse_reg(args[0]), parse_reg(args[1]), parse_imm(args[2])),
    'sltiu': lambda args: SLTIU(parse_reg(args[0]), parse_reg(args[1]), parse_imm(args[2])),
    'xori': lambda args: XORI(parse_reg(args[0]), parse_reg(args[1]), parse_imm(args[2])),
    'ori': lambda args: ORI(parse_reg(args[0]), parse_reg(args[1]), parse_imm(args[2])),
    'andi': lambda args: ANDI(parse_reg(args[0]), parse_reg(args[1]), parse_imm(args[2])),
    'slli': lambda args: SLLI(parse_reg(args[0]), parse_reg(args[1]), parse_imm(args[2])),
    'srli': lambda args: SRLI(parse_reg(args[0]), parse_reg(args[1]), parse_imm(args[2])),
    'srai': lambda args: SRAI(parse_reg(args[0]), parse_reg(args[1]), parse_imm(args[2])),
    'add': lambda args: ADD(parse_reg(args[0]), parse_reg(args[1]), parse_reg(args[2])),
    'sub': lambda args: SUB(parse_reg(args[0]), parse_reg(args[1]), parse_reg(args[2])),
    'sll': lambda args: SLL(parse_reg(args[0]), parse_reg(args[1]), parse_reg(args[2])),
    'slt': lambda args: SLT(parse_reg(args[0]), parse_reg(args[1]), parse_reg(args[2])),
    'sltu': lambda args: SLTU(parse_reg(args[0]), parse_reg(args[1]), parse_reg(args[2])),
    'xor': lambda args: XOR(parse_reg(args[0]), parse_reg(args[1]), parse_reg(args[2])),
    'srl': lambda args: SRL(parse_reg(args[0]), parse_reg(args[1]), parse_reg(args[2])),
    'sra': lambda args: SRA(parse_reg(args[0]), parse_reg(args[1]), parse_reg(args[2])),
    'or': lambda args: OR(parse_reg(args[0]), parse_reg(args[1]), parse_reg(args[2])),
    'and': lambda args: AND(parse_reg(args[0]), parse_reg(args[1]), parse_reg(args[2])),
    'lw': lambda args: LW(parse_reg(args[0]), parse_reg(args[1]), parse_imm(args[2]) if len(args) > 2 else 0),
    'sw': lambda args: SW(parse_reg(args[0]), parse_reg(args[1]), parse_imm(args[2]) if len(args) > 2 else 0),
    'nop': lambda args: NOP(),
    'ebreak': lambda args: rv32i.EBREAK(),
    'lui': lambda args: rv32i.LUI(parse_reg(args[0]), parse_imm(args[1])),
    'auipc': lambda args: rv32i.AUIPC(parse_reg(args[0]), parse_imm(args[1])),
    'jal': lambda args: rv32i.JAL(parse_reg(args[0]), parse_imm(args[1])),
    'jalr': lambda args: rv32i.JALR(parse_reg(args[0]), parse_reg(args[1]), parse_imm(args[2]) if len(args) > 2 else 0),
    'beq': lambda args: rv32i.BEQ(parse_reg(args[0]), parse_reg(args[1]), parse_imm(args[2])),
    'bne': lambda args: rv32i.BNE(parse_reg(args[0]), parse_reg(args[1]), parse_imm(args[2])),
    'blt': lambda args: rv32i.BLT(parse_reg(args[0]), parse_reg(args[1]), parse_imm(args[2])),
    'bge': lambda args: rv32i.BGE(parse_reg(args[0]), parse_reg(args[1]), parse_imm(args[2])),
    'bltu': lambda args: rv32i.BLTU(parse_reg(args[0]), parse_reg(args[1]), parse_imm(args[2])),
    'bgeu': lambda args: rv32i.BGEU(parse_reg(args[0]), parse_reg(args[1]), parse_imm(args[2])),
    'lb': lambda args: rv32i.LB(parse_reg(args[0]), parse_reg(args[1]), parse_imm(args[2]) if len(args) > 2 else 0),
    'lh': lambda args: rv32i.LH(parse_reg(args[0]), parse_reg(args[1]), parse_imm(args[2]) if len(args) > 2 else 0),
    'lbu': lambda args: rv32i.LBU(parse_reg(args[0]), parse_reg(args[1]), parse_imm(args[2]) if len(args) > 2 else 0),
    'lhu': lambda args: rv32i.LHU(parse_reg(args[0]), parse_reg(args[1]), parse_imm(args[2]) if len(args) > 2 else 0),
    'sb': lambda args: rv32i.SB(parse_reg(args[0]), parse_reg(args[1]), parse_imm(args[2]) if len(args) > 2 else 0),
    'sh': lambda args: rv32i.SH(parse_reg(args[0]), parse_reg(args[1]), parse_imm(args[2]) if len(args) > 2 else 0),
    'li': lambda args: LI(parse_reg(args[0]), parse_imm(args[1])),
    'mv': lambda args: MV(parse_reg(args[0]), parse_reg(args[1])),
}

def assemble_instruction(line):
    """Assemble a single RISC-V instruction from text"""
    # Remove comments and clean up
    line = line.split('#')[0].strip()
    if not line:
        return None
    
    # Parse instruction and operands
    # Handle both "addi x1, x0, 10" and "addi x1 x0 10" formats
    line = line.replace(',', ' ')
    parts = line.lower().split()
    
    if not parts:
        return None
    
    mnemonic = parts[0]
    args = parts[1:]
    
    if mnemonic not in INSTR_MAP:
        raise ValueError(f"Unknown instruction: {mnemonic}")
    
    return INSTR_MAP[mnemonic](args)


def disassemble_instruction(instr):
    """Disassemble a 32-bit instruction to text"""
    opcode = instr & 0x7F
    rd = (instr >> 7) & 0x1F
    funct3 = (instr >> 12) & 0x7
    rs1 = (instr >> 15) & 0x1F
    rs2 = (instr >> 20) & 0x1F
    funct7 = (instr >> 25) & 0x7F
    imm_i = (instr >> 20) & 0xFFF
    if imm_i & 0x800:  # Sign extend
        imm_i |= 0xFFFFF000
        imm_i = imm_i - 0x100000000
    
    if opcode == 0b0010011:  # I-type ALU
        names = {0: 'addi', 2: 'slti', 3: 'sltiu', 4: 'xori', 6: 'ori', 7: 'andi', 1: 'slli', 5: 'srli/srai'}
        name = names.get(funct3, '???')
        if funct3 == 5:
            name = 'srai' if (instr >> 30) & 1 else 'srli'
            imm_i = imm_i & 0x1F
        if funct3 == 1:
            imm_i = imm_i & 0x1F
        return f"{name} x{rd}, x{rs1}, {imm_i}"
    elif opcode == 0b0110011:  # R-type
        names = {
            (0, 0): 'add', (0, 0x20): 'sub', (1, 0): 'sll', (2, 0): 'slt',
            (3, 0): 'sltu', (4, 0): 'xor', (5, 0): 'srl', (5, 0x20): 'sra',
            (6, 0): 'or', (7, 0): 'and'
        }
        name = names.get((funct3, funct7), '???')
        return f"{name} x{rd}, x{rs1}, x{rs2}"
    elif opcode == 0b0000011:  # Load
        return f"lw x{rd}, {imm_i}(x{rs1})"
    elif opcode == 0b0100011:  # Store
        imm_s = ((instr >> 25) << 5) | ((instr >> 7) & 0x1F)
        if imm_s & 0x800:
            imm_s |= 0xFFFFF000
            imm_s = imm_s - 0x100000000
        return f"sw x{rs2}, {imm_s}(x{rs1})"
    elif instr == rv32i.EBREAK():
        return "ebreak"
    else:
        return f"0x{instr:08X} (opcode 0b{opcode:07b})"


# =============================================================================
# RISC-V FPGA Tester Class
# =============================================================================

class RISCVTester:
    """RISC-V FPGA tester via UART"""
    
    CMD_WRITE_INSTR = 0xAA
    CMD_READ_REG    = 0xBB
    CMD_RUN         = 0xCC
    CMD_HALT        = 0xDD
    CMD_RESET       = 0xEE
    CMD_READ_CTR    = 0xBC
    CTR_CYCLES, CTR_INSTRET, CTR_HALTED = 0, 1, 2
    
    def __init__(self, port, baudrate=115200, timeout=1.0):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(0.1)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.program = []  # Store loaded program
    
    def close(self):
        self.ser.close()
    
    def write_instruction(self, addr, instr):
        """Write a 32-bit instruction to memory at specified address"""
        cmd = bytes([
            self.CMD_WRITE_INSTR,
            (addr >> 8) & 0xFF,
            addr & 0xFF,
            (instr >> 24) & 0xFF,
            (instr >> 16) & 0xFF,
            (instr >> 8) & 0xFF,
            instr & 0xFF
        ])
        self.ser.write(cmd)
        ack = self.ser.read(1)
        if len(ack) == 0:
            raise TimeoutError("Timeout waiting for write ACK")
        if ack[0] != self.CMD_WRITE_INSTR:
            raise ValueError(f"Unexpected ACK: {hex(ack[0])}")
        return True
    
    def read_register(self, reg):
        """Read a register value (returns 32-bit integer)"""
        cmd = bytes([self.CMD_READ_REG, reg & 0x1F])
        self.ser.write(cmd)
        data = self.ser.read(4)
        if len(data) != 4:
            raise TimeoutError(f"Timeout reading register (got {len(data)} bytes)")
        return (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]
    
    def read_counter(self, idx):
        """Read a status/performance counter (0 cycles, 1 instret, 2 halted)"""
        self.ser.write(bytes([self.CMD_READ_CTR, idx & 0x3]))
        data = self.ser.read(4)
        if len(data) != 4:
            raise TimeoutError(f"Timeout reading counter (got {len(data)} bytes)")
        return (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]

    def run_to_ebreak(self, program, timeout=2.0, verbose=False):
        """Load a program ending in EBREAK, run it until the core halts and
        return (registers, cycles, instret) read from the halted core."""
        self.reset_processor()
        self.halt_processor()
        self.load_program(program, verbose=verbose)
        self.run_processor()
        deadline = time.time() + timeout
        while not self.read_counter(self.CTR_HALTED):
            if time.time() > deadline:
                raise TimeoutError("processor did not reach EBREAK")
            time.sleep(0.005)
        cycles = self.read_counter(self.CTR_CYCLES)
        instret = self.read_counter(self.CTR_INSTRET)
        regs = self.read_all_registers()
        self.halt_processor()
        return regs, cycles, instret

    def read_all_registers(self):
        """Read all 32 registers"""
        regs = {}
        for i in range(32):
            regs[i] = self.read_register(i)
        return regs
    
    def run_processor(self):
        """Start the processor"""
        self.ser.write(bytes([self.CMD_RUN]))
        ack = self.ser.read(1)
        if len(ack) == 0:
            raise TimeoutError("Timeout waiting for run ACK")
        return ack[0] == self.CMD_RUN
    
    def halt_processor(self):
        """Halt the processor"""
        self.ser.write(bytes([self.CMD_HALT]))
        ack = self.ser.read(1)
        if len(ack) == 0:
            raise TimeoutError("Timeout waiting for halt ACK")
        return ack[0] == self.CMD_HALT
    
    def reset_processor(self):
        """Reset the processor (clears registers and PC)"""
        self.ser.write(bytes([self.CMD_RESET]))
        ack = self.ser.read(1)
        if len(ack) == 0:
            raise TimeoutError("Timeout waiting for reset ACK")
        return ack[0] == self.CMD_RESET
    
    def load_program(self, program, start_addr=0, verbose=True):
        """Load a list of instructions into memory.

        An EBREAK is appended if the program has none, so execution stops at
        the end of the program instead of running into stale memory.
        """
        if rv32i.EBREAK() not in program:
            program = list(program) + [rv32i.EBREAK()]
        self.program = program
        for i, instr in enumerate(program):
            addr = start_addr + i * 4
            self.write_instruction(addr, instr)
            if verbose:
                disasm = disassemble_instruction(instr)
                print(f"  [{addr:04X}] {instr:08X}  {disasm}")
    
    def execute_program(self, program, run_time=0.1, verbose=True):
        """Load and execute a program (EBREAK appended), return final registers"""
        if verbose:
            print("Loading and running program...")
        regs, _, _ = self.run_to_ebreak(list(program) + [rv32i.EBREAK()],
                                        timeout=max(run_time, 1.0), verbose=verbose)
        return regs


# =============================================================================
# Test Functions
# =============================================================================

def find_icebreaker():
    """Find iCEBreaker serial port"""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if 'iCEBreaker' in port.description or 'FTDI' in port.description:
            return port.device
        if 'ttyUSB' in port.device or 'ttyACM' in port.device:
            return port.device
    return None


def test_basic_alu(tester):
    """Test basic ALU operations (ADDI, ADD)"""
    print("\n" + "=" * 60)
    print("TEST: Basic ALU Operations")
    print("=" * 60)
    
    tester.reset_processor()
    time.sleep(0.01)
    tester.halt_processor()
    
    # Test: x1 = 10, x2 = 25, x3 = x1 + x2 = 35
    program = [
        ADDI(1, 0, 10),   # x1 = 10
        ADDI(2, 0, 25),   # x2 = 25
        ADD(3, 1, 2),     # x3 = x1 + x2
        NOP(), NOP(), NOP(), NOP(), NOP()
    ]
    
    print("\nProgram:")
    tester.load_program(program)
    
    print("\nExecuting...")
    tester.run_processor()
    time.sleep(0.1)
    tester.halt_processor()
    
    print("\nResults:")
    checks = [(1, 10), (2, 25), (3, 35)]
    passed = True
    for reg, expected in checks:
        val = tester.read_register(reg)
        status = "✓" if val == expected else "✗"
        print(f"  x{reg} = {val:5d} (expected {expected:5d}) [{status}]")
        if val != expected:
            passed = False
    
    return passed


def test_arithmetic(tester):
    """Test arithmetic operations (ADD, SUB, ADDI with negative)"""
    print("\n" + "=" * 60)
    print("TEST: Arithmetic Operations")
    print("=" * 60)
    
    tester.reset_processor()
    time.sleep(0.01)
    tester.halt_processor()
    
    program = [
        ADDI(1, 0, 100),    # x1 = 100
        ADDI(2, 0, 37),     # x2 = 37
        ADD(3, 1, 2),       # x3 = 100 + 37 = 137
        SUB(4, 1, 2),       # x4 = 100 - 37 = 63
        ADDI(5, 0, -10),    # x5 = -10 (0xFFFFFFF6)
        ADD(6, 1, 5),       # x6 = 100 + (-10) = 90
        NOP(), NOP(), NOP(), NOP()
    ]
    
    print("\nProgram:")
    tester.load_program(program)
    
    print("\nExecuting...")
    tester.run_processor()
    time.sleep(0.1)
    tester.halt_processor()
    
    print("\nResults:")
    checks = [
        (1, 100, "100"),
        (2, 37, "37"),
        (3, 137, "100 + 37"),
        (4, 63, "100 - 37"),
        (5, 0xFFFFFFF6, "-10"),
        (6, 90, "100 + (-10)")
    ]
    passed = True
    for reg, expected, desc in checks:
        val = tester.read_register(reg)
        status = "✓" if val == expected else "✗"
        if expected > 0x7FFFFFFF:
            print(f"  x{reg} = 0x{val:08X} (expected 0x{expected:08X}, {desc}) [{status}]")
        else:
            print(f"  x{reg} = {val:5d} (expected {expected:5d}, {desc}) [{status}]")
        if val != expected:
            passed = False
    
    return passed


def test_logical(tester):
    """Test logical operations (AND, OR, XOR, ANDI, ORI, XORI)"""
    print("\n" + "=" * 60)
    print("TEST: Logical Operations")
    print("=" * 60)
    
    tester.reset_processor()
    time.sleep(0.01)
    tester.halt_processor()
    
    program = [
        ADDI(1, 0, 0xFF),    # x1 = 0xFF (255)
        ADDI(2, 0, 0x0F),    # x2 = 0x0F (15)
        AND(3, 1, 2),        # x3 = 0xFF & 0x0F = 0x0F
        OR(4, 1, 2),         # x4 = 0xFF | 0x0F = 0xFF
        XOR(5, 1, 2),        # x5 = 0xFF ^ 0x0F = 0xF0
        ANDI(6, 1, 0x55),    # x6 = 0xFF & 0x55 = 0x55
        ORI(7, 2, 0xF0),     # x7 = 0x0F | 0xF0 = 0xFF
        XORI(8, 1, 0xFF),    # x8 = 0xFF ^ 0xFF = 0x00
        NOP(), NOP(), NOP(), NOP()
    ]
    
    print("\nProgram:")
    tester.load_program(program)
    
    print("\nExecuting...")
    tester.run_processor()
    time.sleep(0.1)
    tester.halt_processor()
    
    print("\nResults:")
    checks = [
        (1, 0xFF, "0xFF"),
        (2, 0x0F, "0x0F"),
        (3, 0x0F, "0xFF & 0x0F"),
        (4, 0xFF, "0xFF | 0x0F"),
        (5, 0xF0, "0xFF ^ 0x0F"),
        (6, 0x55, "0xFF & 0x55"),
        (7, 0xFF, "0x0F | 0xF0"),
        (8, 0x00, "0xFF ^ 0xFF"),
    ]
    passed = True
    for reg, expected, desc in checks:
        val = tester.read_register(reg)
        status = "✓" if val == expected else "✗"
        print(f"  x{reg} = 0x{val:02X} (expected 0x{expected:02X}, {desc}) [{status}]")
        if val != expected:
            passed = False
    
    return passed


def test_shifts(tester):
    """Test shift operations (SLL, SRL, SRA, SLLI, SRLI, SRAI)"""
    print("\n" + "=" * 60)
    print("TEST: Shift Operations")
    print("=" * 60)
    
    tester.reset_processor()
    time.sleep(0.01)
    tester.halt_processor()
    
    program = [
        ADDI(1, 0, 1),        # x1 = 1
        SLLI(2, 1, 4),        # x2 = 1 << 4 = 16
        SLLI(3, 1, 8),        # x3 = 1 << 8 = 256
        ADDI(4, 0, 256),      # x4 = 256
        SRLI(5, 4, 4),        # x5 = 256 >> 4 = 16
        ADDI(6, 0, -16),      # x6 = -16 (0xFFFFFFF0)
        SRLI(7, 6, 4),        # x7 = logical shift: 0x0FFFFFFF
        SRAI(8, 6, 4),        # x8 = arithmetic shift: 0xFFFFFFFF (-1)
        NOP(), NOP(), NOP(), NOP()
    ]
    
    print("\nProgram:")
    tester.load_program(program)
    
    print("\nExecuting...")
    tester.run_processor()
    time.sleep(0.1)
    tester.halt_processor()
    
    print("\nResults:")
    checks = [
        (1, 1, "1"),
        (2, 16, "1 << 4"),
        (3, 256, "1 << 8"),
        (4, 256, "256"),
        (5, 16, "256 >> 4"),
        (6, 0xFFFFFFF0, "-16"),
        (7, 0x0FFFFFFF, "-16 logical >> 4"),
        (8, 0xFFFFFFFF, "-16 arith >> 4"),
    ]
    passed = True
    for reg, expected, desc in checks:
        val = tester.read_register(reg)
        status = "✓" if val == expected else "✗"
        print(f"  x{reg} = 0x{val:08X} (expected 0x{expected:08X}, {desc}) [{status}]")
        if val != expected:
            passed = False
    
    return passed


def test_comparison(tester):
    """Test comparison operations (SLT, SLTU, SLTI, SLTIU)"""
    print("\n" + "=" * 60)
    print("TEST: Comparison Operations")
    print("=" * 60)
    
    tester.reset_processor()
    time.sleep(0.01)
    tester.halt_processor()
    
    program = [
        ADDI(1, 0, 10),       # x1 = 10
        ADDI(2, 0, 20),       # x2 = 20
        ADDI(3, 0, -5),       # x3 = -5
        SLT(4, 1, 2),         # x4 = (10 < 20) = 1
        SLT(5, 2, 1),         # x5 = (20 < 10) = 0
        SLT(6, 3, 1),         # x6 = (-5 < 10) = 1 (signed)
        SLTU(7, 3, 1),        # x7 = (0xFFFFFFFB < 10) = 0 (unsigned, -5 is large)
        SLTI(8, 1, 15),       # x8 = (10 < 15) = 1
        SLTI(9, 1, 5),        # x9 = (10 < 5) = 0
        NOP(), NOP(), NOP(), NOP()
    ]
    
    print("\nProgram:")
    tester.load_program(program)
    
    print("\nExecuting...")
    tester.run_processor()
    time.sleep(0.1)
    tester.halt_processor()
    
    print("\nResults:")
    checks = [
        (1, 10, "10"),
        (2, 20, "20"),
        (3, 0xFFFFFFFB, "-5"),
        (4, 1, "10 < 20 (signed)"),
        (5, 0, "20 < 10 (signed)"),
        (6, 1, "-5 < 10 (signed)"),
        (7, 0, "-5 < 10 (unsigned)"),
        (8, 1, "10 < 15"),
        (9, 0, "10 < 5"),
    ]
    passed = True
    for reg, expected, desc in checks:
        val = tester.read_register(reg)
        status = "✓" if val == expected else "✗"
        if expected > 0x7FFFFFFF:
            print(f"  x{reg} = 0x{val:08X} (expected 0x{expected:08X}, {desc}) [{status}]")
        else:
            print(f"  x{reg} = {val} (expected {expected}, {desc}) [{status}]")
        if val != expected:
            passed = False
    
    return passed


def test_all_registers(tester):
    """Test writing to all registers"""
    print("\n" + "=" * 60)
    print("TEST: All Registers (x1-x31)")
    print("=" * 60)
    
    tester.reset_processor()
    time.sleep(0.01)
    tester.halt_processor()
    
    # Write unique values to x1-x31
    program = []
    for i in range(1, 32):
        program.append(ADDI(i, 0, i * 7))  # x_i = i * 7
    program.extend([NOP()] * 8)
    
    print("\nProgram: ADDI xi, x0, i*7 for i=1..31")
    tester.load_program(program, verbose=False)
    print(f"  Loaded {len(program)} instructions")
    
    print("\nExecuting...")
    tester.run_processor()
    time.sleep(0.3)
    tester.halt_processor()
    
    print("\nResults:")
    passed = True
    for i in range(32):
        val = tester.read_register(i)
        expected = 0 if i == 0 else i * 7
        status = "✓" if val == expected else "✗"
        if i == 0 or val != expected:
            print(f"  x{i:2d} = {val:5d} (expected {expected:5d}) [{status}]")
        if val != expected:
            passed = False
    
    if passed:
        print("  All 32 registers verified correctly!")
    
    return passed


def test_data_forwarding(tester):
    """Test that back-to-back instructions work (data forwarding or hazard handling)"""
    print("\n" + "=" * 60)
    print("TEST: Data Forwarding / Pipeline Hazards")
    print("=" * 60)
    
    tester.reset_processor()
    time.sleep(0.01)
    tester.halt_processor()
    
    # Chain of dependencies
    program = [
        ADDI(1, 0, 1),    # x1 = 1
        ADDI(2, 1, 1),    # x2 = x1 + 1 = 2
        ADDI(3, 2, 1),    # x3 = x2 + 1 = 3
        ADDI(4, 3, 1),    # x4 = x3 + 1 = 4
        ADDI(5, 4, 1),    # x5 = x4 + 1 = 5
        ADD(6, 1, 5),     # x6 = x1 + x5 = 6
        ADD(7, 2, 4),     # x7 = x2 + x4 = 6
        ADD(8, 3, 3),     # x8 = x3 + x3 = 6
        NOP(), NOP(), NOP(), NOP()
    ]
    
    print("\nProgram (chain of dependencies):")
    tester.load_program(program)
    
    print("\nExecuting...")
    tester.run_processor()
    time.sleep(0.1)
    tester.halt_processor()
    
    print("\nResults:")
    checks = [(1,1), (2,2), (3,3), (4,4), (5,5), (6,6), (7,6), (8,6)]
    passed = True
    for reg, expected in checks:
        val = tester.read_register(reg)
        status = "✓" if val == expected else "✗"
        print(f"  x{reg} = {val} (expected {expected}) [{status}]")
        if val != expected:
            passed = False
    
    return passed


# =============================================================================
# Interactive Mode
# =============================================================================

def interactive_mode(tester):
    """Full-featured interactive mode"""
    print("\n" + "=" * 70)
    print("RISC-V Interactive Mode")
    print("=" * 70)
    print("""
Commands:
  Assembly & Execution:
    asm <instr>           - Assemble and show encoding (e.g., 'asm addi x1, x0, 42')
    load <instr> [...]    - Load instruction(s) and auto-add NOPs
    run [ms]              - Run processor for specified time (default 100ms)
    step                  - Load, run briefly, and show registers
    
  Direct Control:
    write <addr> <hex>    - Write raw hex instruction to address
    reset                 - Reset processor (clears all registers)
    halt                  - Halt processor
    start                 - Start processor (continuous run)
    
  Inspection:
    reg [n]               - Read register(s) (all if no arg, or specific)
    regs                  - Show all non-zero registers
    dump                  - Show all registers in table format
    
  Program Management:
    prog                  - Show current loaded program
    clear                 - Clear program buffer
    example [name]        - Load example program (fibonacci, counter, etc.)
    
  Help:
    help                  - Show this help
    quit / q              - Exit interactive mode

Examples:
  > asm addi x1, x0, 10
  > load addi x1, x0, 10
  > load addi x2, x0, 20
  > load add x3, x1, x2
  > run
  > regs
""")
    
    program_buffer = []
    
    while True:
        try:
            line = input("\n> ").strip()
            if not line:
                continue
            
            parts = line.split(maxsplit=1)
            cmd = parts[0].lower()
            args = parts[1] if len(parts) > 1 else ""
            
            # === Quit ===
            if cmd in ['quit', 'q', 'exit']:
                print("Goodbye!")
                break
            
            # === Help ===
            elif cmd == 'help':
                print("See command list above. Try: asm addi x1, x0, 42")
            
            # === Assemble ===
            elif cmd == 'asm':
                try:
                    instr = assemble_instruction(args)
                    if instr is not None:
                        print(f"  Encoding: 0x{instr:08X}")
                        print(f"  Binary:   {instr:032b}")
                        print(f"  Disasm:   {disassemble_instruction(instr)}")
                except Exception as e:
                    print(f"  Error: {e}")
            
            # === Load instruction ===
            elif cmd == 'load':
                try:
                    instr = assemble_instruction(args)
                    if instr is not None:
                        program_buffer.append(instr)
                        addr = (len(program_buffer) - 1) * 4
                        print(f"  [{addr:04X}] {instr:08X}  {disassemble_instruction(instr)}")
                        print(f"  Program now has {len(program_buffer)} instruction(s)")
                except Exception as e:
                    print(f"  Error: {e}")
            
            # === Run ===
            elif cmd == 'run':
                run_ms = int(args) if args else 100
                
                if not program_buffer:
                    print("  No program loaded. Use 'load' to add instructions.")
                    continue
                
                # Add NOPs if needed
                prog = program_buffer.copy()
                while len(prog) < 8 or len(prog) % 4 != 0:
                    prog.append(NOP())
                
                print(f"  Resetting and loading {len(prog)} instructions...")
                tester.reset_processor()
                time.sleep(0.01)
                tester.halt_processor()
                tester.load_program(prog, verbose=False)
                
                print(f"  Running for {run_ms}ms...")
                tester.run_processor()
                time.sleep(run_ms / 1000.0)
                tester.halt_processor()
                
                print("  Done! Use 'regs' to see results.")
            
            # === Step (load + short run + show) ===
            elif cmd == 'step':
                if not program_buffer:
                    print("  No program loaded.")
                    continue
                
                prog = program_buffer.copy()
                while len(prog) < 8:
                    prog.append(NOP())
                
                tester.reset_processor()
                time.sleep(0.01)
                tester.halt_processor()
                tester.load_program(prog, verbose=False)
                tester.run_processor()
                time.sleep(0.1)
                tester.halt_processor()
                
                # Show non-zero registers
                print("\n  Registers (non-zero):")
                for i in range(32):
                    val = tester.read_register(i)
                    if val != 0:
                        print(f"    x{i:2d} = {val:10d} (0x{val:08X})")
            
            # === Write raw instruction ===
            elif cmd == 'write':
                try:
                    parts = args.split()
                    addr = int(parts[0], 16)
                    instr = int(parts[1], 16)
                    tester.write_instruction(addr, instr)
                    print(f"  Wrote 0x{instr:08X} to address 0x{addr:04X}")
                except Exception as e:
                    print(f"  Error: {e}")
                    print("  Usage: write <addr_hex> <instr_hex>")
            
            # === Reset ===
            elif cmd == 'reset':
                tester.reset_processor()
                print("  Processor reset (registers cleared)")
            
            # === Halt ===
            elif cmd == 'halt':
                tester.halt_processor()
                print("  Processor halted")
            
            # === Start ===
            elif cmd == 'start':
                tester.run_processor()
                print("  Processor started (running continuously)")
            
            # === Read register(s) ===
            elif cmd == 'reg':
                if args:
                    reg = parse_reg(args)
                    val = tester.read_register(reg)
                    print(f"  x{reg} = {val} (0x{val:08X})")
                else:
                    for i in range(32):
                        val = tester.read_register(i)
                        print(f"  x{i:2d} = {val:10d} (0x{val:08X})")
            
            # === Show non-zero registers ===
            elif cmd == 'regs':
                print("  Non-zero registers:")
                found = False
                for i in range(32):
                    val = tester.read_register(i)
                    if val != 0 or i == 0:
                        print(f"    x{i:2d} = {val:10d} (0x{val:08X})")
                        found = True
                if not found:
                    print("    (all registers are zero)")
            
            # === Dump all registers ===
            elif cmd == 'dump':
                print("  Register Dump:")
                print("  " + "-" * 50)
                for row in range(8):
                    line = "  "
                    for col in range(4):
                        i = row * 4 + col
                        val = tester.read_register(i)
                        line += f"x{i:2d}={val:08X}  "
                    print(line)
            
            # === Show program ===
            elif cmd == 'prog':
                if not program_buffer:
                    print("  No program loaded.")
                else:
                    print(f"  Current program ({len(program_buffer)} instructions):")
                    for i, instr in enumerate(program_buffer):
                        addr = i * 4
                        print(f"    [{addr:04X}] {instr:08X}  {disassemble_instruction(instr)}")
            
            # === Clear program ===
            elif cmd == 'clear':
                program_buffer = []
                print("  Program buffer cleared.")
            
            # === Example programs ===
            elif cmd == 'example':
                name = args.lower() if args else 'list'
                
                examples = {
                    'counter': [
                        ADDI(1, 0, 0),    # x1 = 0 (counter)
                        ADDI(2, 0, 10),   # x2 = 10 (limit)
                        ADDI(1, 1, 1),    # x1 += 1
                    ],
                    'fibonacci': [
                        ADDI(1, 0, 1),    # x1 = 1 (fib[0])
                        ADDI(2, 0, 1),    # x2 = 1 (fib[1])
                        ADD(3, 1, 2),     # x3 = x1 + x2 = 2
                        ADD(4, 2, 3),     # x4 = x2 + x3 = 3
                        ADD(5, 3, 4),     # x5 = x3 + x4 = 5
                        ADD(6, 4, 5),     # x6 = x4 + x5 = 8
                        ADD(7, 5, 6),     # x7 = x5 + x6 = 13
                        ADD(8, 6, 7),     # x8 = x6 + x7 = 21
                    ],
                    'bitwise': [
                        ADDI(1, 0, 0xFF),   # x1 = 255
                        ADDI(2, 0, 0x0F),   # x2 = 15
                        AND(3, 1, 2),       # x3 = AND
                        OR(4, 1, 2),        # x4 = OR
                        XOR(5, 1, 2),       # x5 = XOR
                        SLLI(6, 2, 4),      # x6 = x2 << 4
                    ],
                    'arithmetic': [
                        ADDI(1, 0, 100),    # x1 = 100
                        ADDI(2, 0, 37),     # x2 = 37
                        ADD(3, 1, 2),       # x3 = 137
                        SUB(4, 1, 2),       # x4 = 63
                        ADDI(5, 0, -20),    # x5 = -20
                        ADD(6, 1, 5),       # x6 = 80
                    ],
                    'shifts': [
                        ADDI(1, 0, 1),      # x1 = 1
                        SLLI(2, 1, 4),      # x2 = 1 << 4 = 16
                        SLLI(3, 1, 8),      # x3 = 1 << 8 = 256
                        ADDI(4, 0, 256),    # x4 = 256
                        SRLI(5, 4, 4),      # x5 = 256 >> 4 = 16
                        ADDI(6, 0, -16),    # x6 = -16
                        SRLI(7, 6, 4),      # x7 = logical shift = 0x0FFFFFFF
                        SRAI(8, 6, 4),      # x8 = arith shift = -1
                    ],
                    'shifts_variable': [
                        ADDI(1, 0, 128),    # x1 = 128
                        ADDI(2, 0, 3),      # x2 = 3 (shift amount)
                        SLL(3, 1, 2),       # x3 = 128 << 3 = 1024
                        SRL(4, 1, 2),       # x4 = 128 >> 3 = 16
                        ADDI(5, 0, -128),   # x5 = -128
                        SRA(6, 5, 2),       # x6 = -128 >>> 3 = -16
                        ADDI(7, 0, 7),      # x7 = 7
                        SLL(8, 1, 7),       # x8 = 128 << 7 = 16384
                    ],
                    'comparisons': [
                        ADDI(1, 0, 10),     # x1 = 10
                        ADDI(2, 0, 20),     # x2 = 20
                        SLT(3, 1, 2),       # x3 = (10 < 20) = 1
                        SLT(4, 2, 1),       # x4 = (20 < 10) = 0
                        ADDI(5, 0, -5),     # x5 = -5
                        SLT(6, 5, 1),       # x6 = (-5 < 10) signed = 1
                        SLTU(7, 5, 1),      # x7 = (-5 < 10) unsigned = 0
                        SLTI(8, 1, 15),     # x8 = (10 < 15) = 1
                        SLTIU(9, 1, 5),     # x9 = (10 < 5) = 0
                    ],
                    'logical_immediate': [
                        ADDI(1, 0, 0xFF),   # x1 = 255
                        ANDI(2, 1, 0xAA),   # x2 = 255 & 170 = 170
                        ANDI(3, 1, 0x55),   # x3 = 255 & 85 = 85
                        ORI(4, 0, 0xF0),    # x4 = 0 | 240 = 240
                        XORI(5, 1, 0xFF),   # x5 = 255 ^ 255 = 0
                        XORI(6, 1, 0xAA),   # x6 = 255 ^ 170 = 85
                        ORI(7, 2, 3),       # x7 = 170 | 3 = 171
                    ],
                    'logical_register': [
                        ADDI(1, 0, 0xFF),   # x1 = 255 (0b11111111)
                        ADDI(2, 0, 0x0F),   # x2 = 15  (0b00001111)
                        AND(3, 1, 2),       # x3 = 255 & 15 = 15
                        OR(4, 1, 2),        # x4 = 255 | 15 = 255
                        XOR(5, 1, 2),       # x5 = 255 ^ 15 = 240
                        ADDI(6, 0, 0xAA),   # x6 = 170 (0b10101010)
                        AND(7, 1, 6),       # x7 = 255 & 170 = 170
                        OR(8, 2, 6),        # x8 = 15 | 170 = 175
                        XOR(9, 1, 6),       # x9 = 255 ^ 170 = 85
                    ],
                    'powers_of_two': [
                        ADDI(1, 0, 1),      # x1 = 1 = 2^0
                        SLLI(2, 1, 1),      # x2 = 2 = 2^1
                        SLLI(3, 1, 2),      # x3 = 4 = 2^2
                        SLLI(4, 1, 3),      # x4 = 8 = 2^3
                        SLLI(5, 1, 4),      # x5 = 16 = 2^4
                        SLLI(6, 1, 5),      # x6 = 32 = 2^5
                        SLLI(7, 1, 6),      # x7 = 64 = 2^6
                        SLLI(8, 1, 7),      # x8 = 128 = 2^7
                    ],
                    'sign_extension': [
                        ADDI(1, 0, -1),     # x1 = -1 (0xFFFFFFFF)
                        SRLI(2, 1, 4),      # x2 = logical >> = 0x0FFFFFFF
                        SRAI(3, 1, 4),      # x3 = arith >> = 0xFFFFFFFF
                        ADDI(4, 0, -128),   # x4 = -128
                        SRAI(5, 4, 3),      # x5 = -128 >>> 3 = -16
                        ADDI(6, 0, 127),    # x6 = 127
                        SRAI(7, 6, 3),      # x7 = 127 >>> 3 = 15
                        SRLI(8, 4, 3),      # x8 = -128 >> 3 (logical)
                    ],
                    'mixed_operations': [
                        ADDI(1, 0, 15),     # x1 = 15
                        ADDI(2, 0, 7),      # x2 = 7
                        ADD(3, 1, 2),       # x3 = 15 + 7 = 22
                        SUB(4, 1, 2),       # x4 = 15 - 7 = 8
                        AND(5, 1, 2),       # x5 = 15 & 7 = 7
                        OR(6, 1, 2),        # x6 = 15 | 7 = 15
                        XOR(7, 1, 2),       # x7 = 15 ^ 7 = 8
                        SLL(8, 1, 2),       # x8 = 15 << 7 = 1920
                        SRL(9, 1, 2),       # x9 = 15 >> 7 = 0
                        SLT(10, 2, 1),      # x10 = (7 < 15) = 1
                    ],
                    'edge_cases': [
                        ADDI(1, 0, 2047),   # x1 = max 12-bit positive
                        ADDI(2, 0, -2048),  # x2 = min 12-bit negative
                        ADD(3, 1, 1),       # x3 = 4094
                        ADD(4, 2, 2),       # x4 = -4096
                        SUB(5, 2, 1),       # x5 = -2048 - 2047 = -4095
                        ADDI(6, 0, 0),      # x6 = 0
                        ADDI(7, 6, 1),      # x7 = 1
                        SUB(8, 6, 7),       # x8 = 0 - 1 = -1
                    ],
                    'data_hazards': [
                        ADDI(1, 0, 5),      # x1 = 5
                        ADDI(2, 1, 3),      # x2 = x1 + 3 = 8 (RAW hazard)
                        ADDI(3, 2, 2),      # x3 = x2 + 2 = 10 (RAW hazard)
                        ADD(4, 1, 3),       # x4 = x1 + x3 = 15
                        ADD(5, 2, 4),       # x5 = x2 + x4 = 23
                        SUB(6, 5, 1),       # x6 = x5 - x1 = 18
                        ADD(7, 6, 6),       # x7 = x6 + x6 = 36
                    ],
                    'all_instructions': [
                        ADDI(1, 0, 42),     # ADDI: x1 = 42
                        ADDI(2, 0, 13),     # ADDI: x2 = 13
                        ADD(3, 1, 2),       # ADD: x3 = 55
                        SUB(4, 1, 2),       # SUB: x4 = 29
                        AND(5, 1, 2),       # AND: x5 = 8
                        OR(6, 1, 2),        # OR: x6 = 47
                        XOR(7, 1, 2),       # XOR: x7 = 39
                        SLT(8, 2, 1),       # SLT: x8 = 1
                        SLTU(9, 2, 1),      # SLTU: x9 = 1
                        SLLI(10, 2, 2),     # SLLI: x10 = 52
                        SRLI(11, 1, 2),     # SRLI: x11 = 10
                        SRAI(12, 1, 2),     # SRAI: x12 = 10
                        SLTI(13, 1, 50),    # SLTI: x13 = 1
                        SLTIU(14, 1, 50),   # SLTIU: x14 = 1
                        ANDI(15, 1, 0x0F),  # ANDI: x15 = 10
                    ],
                }
                
                if name == 'list' or name not in examples:
                    print("  Available examples:")
                    for n in examples:
                        print(f"    - {n}")
                    print("  Usage: example <name>")
                else:
                    program_buffer = examples[name]
                    print(f"  Loaded '{name}' example ({len(program_buffer)} instructions):")
                    for i, instr in enumerate(program_buffer):
                        print(f"    [{i*4:04X}] {disassemble_instruction(instr)}")
            
            else:
                # Try to interpret as an instruction
                try:
                    instr = assemble_instruction(line)
                    if instr is not None:
                        program_buffer.append(instr)
                        addr = (len(program_buffer) - 1) * 4
                        print(f"  [{addr:04X}] {instr:08X}  {disassemble_instruction(instr)}")
                except:
                    print(f"  Unknown command: {cmd}")
                    print("  Type 'help' for available commands")
                    
        except KeyboardInterrupt:
            print("\n  (Use 'quit' to exit)")
        except Exception as e:
            print(f"  Error: {e}")


# =============================================================================
# Reference-model tests (no NOP padding: exercise forwarding, stalls, flushes)
# =============================================================================

def _compare_with_model(tester, program, label):
    ref = rv32i.ISS(program)
    ref.run()
    regs, cycles, instret = tester.run_to_ebreak(program)
    bad = [r for r in range(32) if regs[r] != ref.x[r]]
    for r in bad:
        print(f"  {label}: x{r} = 0x{regs[r]:08X} (expected 0x{ref.x[r]:08X}) ✗")
    if instret != ref.retired:
        print(f"  {label}: instret {instret} (expected {ref.retired}) ✗")
    return not bad and instret == ref.retired, cycles, instret


def test_control_flow(tester):
    """Branches (all six conditions), JAL/JALR and load-use, back-to-back"""
    print("\n" + "=" * 60)
    print("TEST: Control Flow + Load-Use Hazards (no NOPs)")
    print("=" * 60)
    R = rv32i
    prog = [R.ADDI(1, 0, -1), R.ADDI(2, 0, 1)]
    for i, fn in enumerate(R.BRANCHES):
        prog += [fn(1, 2, 8), R.ADDI(10 + i, 0, 1), fn(2, 1, 8), R.ADDI(20 + i, 0, 1)]
    prog += [R.ADDI(3, 0, 0x55), R.SW(3, 0, 0x10), R.LW(4, 0, 0x10), R.ADDI(5, 4, 1),
             R.LW(6, 0, 0x10), R.BEQ(6, 3, 8), R.ADDI(7, 0, 1),
             R.JAL(8, 12), R.ADDI(9, 0, 1), R.EBREAK(), R.JALR(0, 8, 4), R.EBREAK()]
    passed, _, _ = _compare_with_model(tester, prog, "control-flow")
    print("  ✓ matches reference model" if passed else "  ✗ mismatch")
    return passed


def test_random_vs_model(tester, count=50, seed=2026):
    """Random RV32I programs compared against the reference model"""
    print("\n" + "=" * 60)
    print(f"TEST: {count} Random Programs vs Reference Model")
    print("=" * 60)
    R = rv32i
    rng = random.Random(seed)
    regs_pool = list(range(1, 8))
    clear = [R.SW(0, 9, off) for off in range(0, 128, 4)]   # dmem persists between runs
    failures = 0
    for n in range(count):
        while True:
            body = [R.ADDI(rng.choice(regs_pool), rng.choice(regs_pool + [0]), rng.randint(-2048, 2047))]
            while len(body) < 60:
                k = rng.random()
                rd, a, b = rng.choice(regs_pool), rng.choice(regs_pool + [0]), rng.choice(regs_pool + [0])
                if k < 0.4:   body.append(rng.choice(R.ALU_RR)(rd, a, b))
                elif k < 0.6: body.append(rng.choice(R.ALU_RI)(rd, a, rng.randint(-2048, 2047)))
                elif k < 0.7: body.append(rng.choice([R.LW, R.LH, R.LBU])(rd, 9, rng.randrange(0, 128, 4)))
                elif k < 0.8: body.append(rng.choice([R.SW, R.SH, R.SB])(a, 9, rng.randrange(0, 128, 4)))
                elif k < 0.9: body.append(rng.choice(R.BRANCHES)(a, b, 4 * rng.randint(1, 4)))
                else:         body.append(R.JAL(rd, 4 * rng.randint(1, 4)))
            prog = [*R.li(9, 0x100), *clear, *body, R.EBREAK(), R.EBREAK(), R.EBREAK(), R.EBREAK()]
            try:
                R.ISS(prog).run(max_steps=2000)
                break
            except RuntimeError:
                continue
        ok, _, _ = _compare_with_model(tester, prog, f"prog {n}")
        failures += not ok
    print(f"  {count - failures}/{count} programs matched the reference model")
    return failures == 0


def test_cpi(tester):
    """Run the benchmark programs and report CPI from the on-chip counters"""
    print("\n" + "=" * 60)
    print("TEST: Benchmarks + Hardware Performance Counters")
    print("=" * 60)
    passed = True
    for name, gen in BENCHMARKS.items():
        ok, cycles, instret = _compare_with_model(tester, gen(), name)
        passed &= ok
        print(f"  {name:16s} cycles={cycles:6d} instret={instret:6d} "
              f"CPI={cycles / instret:.3f} {'✓' if ok else '✗'}")
    return passed


# =============================================================================
# Main
# =============================================================================

def run_all_tests(tester):
    """Run comprehensive test suite"""
    print("\n" + "=" * 70)
    print("RISC-V 5-Stage Pipelined Processor - Comprehensive Test Suite")
    print("=" * 70)
    
    tests = [
        ("Basic ALU (ADD/ADDI)", test_basic_alu),
        ("Arithmetic (ADD/SUB)", test_arithmetic),
        ("Logical (AND/OR/XOR)", test_logical),
        ("Shift Operations", test_shifts),
        ("Comparison (SLT/SLTI)", test_comparison),
        ("Data Forwarding", test_data_forwarding),
        ("All Registers (x1-x31)", test_all_registers),
        ("Control Flow + Load-Use", test_control_flow),
        ("Random vs Reference Model", test_random_vs_model),
        ("Benchmarks + CPI", test_cpi),
    ]
    
    results = []
    for name, test_func in tests:
        try:
            passed = test_func(tester)
            results.append((name, passed))
        except Exception as e:
            print(f"\n  ERROR in {name}: {e}")
            results.append((name, False))
    
    # Summary
    print("\n" + "=" * 70)
    print("TEST SUMMARY")
    print("=" * 70)
    
    all_passed = True
    for name, passed in results:
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"  {name:30s} {status}")
        if not passed:
            all_passed = False
    
    print("=" * 70)
    if all_passed:
        print("🎉 ALL TESTS PASSED! The processor is fully functional.")
    else:
        print("❌ SOME TESTS FAILED. Check the output above for details.")
    print("=" * 70)
    
    return all_passed


def main():
    parser = argparse.ArgumentParser(
        description='RISC-V 5-Stage Pipelined Processor - FPGA Test Suite',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 test_fpga.py /dev/ttyUSB1           # Run all tests
  python3 test_fpga.py /dev/ttyUSB1 -i        # Interactive mode
  python3 test_fpga.py /dev/ttyUSB1 -t alu    # Run specific test
        """
    )
    parser.add_argument('port', nargs='?', default=None, 
                        help='Serial port (auto-detect if not specified)')
    parser.add_argument('-b', '--baud', type=int, default=115200, 
                        help='Baud rate (default: 115200)')
    parser.add_argument('-i', '--interactive', action='store_true', 
                        help='Enter interactive mode')
    parser.add_argument('-t', '--test', 
                        choices=['all', 'alu', 'arith', 'logic', 'shift', 'cmp', 'fwd', 'regs',
                                 'ctrl', 'random', 'cpi'],
                        default='all', help='Specific test to run')
    args = parser.parse_args()
    
    # Find port
    port = args.port or find_icebreaker()
    if not port:
        print("Error: Could not find iCEBreaker. Specify port explicitly.")
        print("\nAvailable ports:")
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device}: {p.description}")
        sys.exit(1)
    
    print(f"Connecting to {port} at {args.baud} baud...")
    
    try:
        tester = RISCVTester(port, args.baud)
        
        if args.interactive:
            interactive_mode(tester)
        elif args.test == 'all':
            success = run_all_tests(tester)
            sys.exit(0 if success else 1)
        else:
            # Run specific test
            test_map = {
                'alu': test_basic_alu,
                'arith': test_arithmetic,
                'logic': test_logical,
                'shift': test_shifts,
                'cmp': test_comparison,
                'fwd': test_data_forwarding,
                'regs': test_all_registers,
                'ctrl': test_control_flow,
                'random': test_random_vs_model,
                'cpi': test_cpi,
            }
            if args.test in test_map:
                passed = test_map[args.test](tester)
                sys.exit(0 if passed else 1)
        
        tester.close()
        
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nInterrupted")
        sys.exit(0)


if __name__ == '__main__':
    main()
