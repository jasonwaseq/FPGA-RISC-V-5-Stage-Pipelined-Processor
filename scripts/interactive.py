#!/usr/bin/env python3
"""Interactive console for the RV32IM SoC on the iCEBreaker.

Type RISC-V assembly, run it on the real core, look at registers and memory,
single-step it, or load a compiled program. Everything goes through the boot
ROM loader, so nothing has to be rebuilt on the FPGA.

    python3 scripts/interactive.py [/dev/ttyUSB1]
    make interactive

How it works: the program is assembled on the host and written to RAM at
0x2000 with an EBREAK after it. The ROM's trap handler saves every register
when the EBREAK (or any other trap) is hit, and the console reads them back.
Single-stepping plants a temporary EBREAK on each possible successor of the
current instruction and resumes through a small stub that restores all the
registers first, so the program runs on the hardware one instruction at a time.
"""
import os
import struct
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rvasm                                            # noqa: E402
from rvasm import ABI, AsmError, assemble, disassemble  # noqa: E402
from rvhost import Board, Exit, Trap, ProtocolError, TRAP_FRAME, RAM_TOP   # noqa: E402

BASE = 0x2000               # user program
STUB = 0x1F000              # register-restore stub used to start and resume
STACK = 0x1E000             # initial sp
ROM_EXIT = 0x0010_0004      # initial ra: a top-level `ret` ends the program
EBREAK = 0x0010_0073
MASK = 0xFFFF_FFFF

HELP = """
Writing a program
  <instruction>         add a line, e.g.  addi a0, zero, 42   or   loop:
  asm <instruction>     show the encoding without adding it
  prog                  list the program with addresses and encodings
  del [n]               delete the last line (or line n)       clear   delete all
  example [name]        load an example program                source <file.s>
Running it on the board
  run                   run from the start until the program ends
  step [n]              execute one (or n) instructions and show what changed
  cont                  continue from where stepping stopped
  restart               go back to the start (registers cleared)
Looking around
  regs                  registers that are not zero            dump   all of them
  reg <name>            one register                           pc     where execution is
  mem <addr> [words]    memory, RAM / ROM / flash / peripherals
  write <addr> <value>  write a word (e.g. write 0x80002000 1 lights the LED)
The rest of the system
  exec <file.bin>       load and run a compiled program (firmware/build/*.bin)
  monitor               reset into the monitor shell stored in flash ('~.' to leave)
  reset                 reset the board                        quit

Registers accept either name: x10 or a0. Numbers are decimal or 0x hex.
Branch targets are labels, absolute addresses, or +8 / -4 style offsets.
"""

EXAMPLES = {
    "sum": ("1 + 2 + ... + 100 with a loop", """
        li   a0, 100
        li   a1, 0
      loop:
        add  a1, a1, a0
        addi a0, a0, -1
        bnez a0, loop
    """),
    "fibonacci": ("first 16 Fibonacci numbers stored at 0x100", """
        li   t0, 0x100
        li   t1, 16
        li   a0, 0
        li   a1, 1
      loop:
        sw   a0, 0(t0)
        add  a2, a0, a1
        mv   a0, a1
        mv   a1, a2
        addi t0, t0, 4
        addi t1, t1, -1
        bnez t1, loop
    """),
    "arithmetic": ("add, subtract, negative numbers", """
        li   x1, 100
        li   x2, 37
        add  x3, x1, x2
        sub  x4, x1, x2
        li   x5, -20
        add  x6, x1, x5
    """),
    "bitwise": ("and, or, xor, shifts", """
        li   x1, 0xFF
        li   x2, 0x0F
        and  x3, x1, x2
        or   x4, x1, x2
        xor  x5, x1, x2
        slli x6, x2, 4
        li   x7, -16
        srli x8, x7, 4
        srai x9, x7, 4
    """),
    "comparisons": ("slt / sltu, signed against unsigned", """
        li   x1, 10
        li   x2, 20
        li   x5, -5
        slt  x3, x1, x2
        slt  x4, x2, x1
        slt  x6, x5, x1
        sltu x7, x5, x1
        slti x8, x1, 15
    """),
    "data_hazards": ("every instruction uses the previous result (bypass network)", """
        li   x1, 5
        addi x2, x1, 3
        addi x3, x2, 2
        add  x4, x1, x3
        add  x5, x2, x4
        sub  x6, x5, x1
        add  x7, x6, x6
    """),
    "muldiv": ("M extension, including division by zero", """
        li   a0, 123456
        li   a1, 789
        mul  a2, a0, a1
        mulhu a3, a0, a1
        div  a4, a0, a1
        rem  a5, a0, a1
        li   t0, -7
        li   t1, 2
        div  a6, t0, t1
        rem  a7, t0, t1
        div  s2, a0, zero
    """),
    "memory": ("bytes, halfwords and words at 0x100", """
        li   t0, 0x100
        li   a0, 0x8081F2F3
        sw   a0, 0(t0)
        lb   a1, 0(t0)
        lbu  a2, 0(t0)
        lh   a3, 2(t0)
        lhu  a4, 2(t0)
        sb   a1, 5(t0)
        lw   a5, 4(t0)
    """),
    "function": ("call and return with the stack", """
        li   a0, 6
        call square
        mv   s1, a0
        li   a0, 9
        call square
        add  s1, s1, a0
        j    done
      square:
        addi sp, sp, -4
        sw   ra, 0(sp)
        mul  a0, a0, a0
        lw   ra, 0(sp)
        addi sp, sp, 4
        ret
      done:
    """),
    "led": ("blink the red LED five times (memory-mapped I/O)", """
        li   t0, 0x80002000
        li   t1, 10
      blink:
        andi t2, t1, 1
        sw   t2, 0(t0)
        li   t3, 600000
      wait:
        addi t3, t3, -1
        bnez t3, wait
        addi t1, t1, -1
        bnez t1, blink
        li   t2, 1
        sw   t2, 0(t0)
    """),
    "uart": ("print 'Hi!' through the UART", """
        li   t0, 0x80000000
        li   a0, 72
        call putc
        li   a0, 105
        call putc
        li   a0, 33
        call putc
        li   a0, 10
        call putc
        j    done
      putc:
        lw   t1, 4(t0)
        andi t1, t1, 2
        beqz t1, putc
        sw   a0, 0(t0)
        ret
      done:
    """),
    "counters": ("read the cycle and instruction counters around a loop", """
        csrr s0, mcycle
        csrr s1, minstret
        li   t0, 1000
      loop:
        addi t0, t0, -1
        bnez t0, loop
        csrr s2, mcycle
        csrr s3, minstret
        sub  a0, s2, s0
        sub  a1, s3, s1
    """),
    "trap": ("load from an unmapped address: a precise access fault", """
        li   a0, 1
        li   t0, 0x40000000
        lw   a1, 0(t0)
        li   a0, 2
    """),
}


def restore_stub(target):
    """Machine code that reloads x1..x31 from the trap frame and jumps to target."""
    lines = ["lui x31, 0x20"]
    lines += [f"lw x{r}, {4 * r - 128}(x31)" for r in range(1, 31)]
    lines += ["lw x31, -4(x31)", f"j {target:#x}"]
    return assemble(lines, STUB)


class Console:
    def __init__(self, board):
        self.board = board
        self.lines = []             # program source
        self.words = []             # assembled program, followed by its EBREAK
        self.loaded = False         # the board holds self.words
        self.regs = None            # register state while a program is in progress
        self.pc = None

    # ---- program ------------------------------------------------------------------
    def build(self):
        self.words = assemble(self.lines + ["ebreak"], BASE)
        return self.words

    @property
    def end_pc(self):
        return BASE + 4 * (len(self.words) - 1)

    def changed(self):
        self.loaded = False
        self.regs = self.pc = None

    def add(self, text):
        if not rvasm.is_label(text):
            self._needs_label(text)         # raises on anything but a forward reference
        self.lines.append(text.strip())
        self.changed()
        if rvasm.is_label(text):
            print(f"  {text.strip()}")
            return
        try:
            words = self.build()
            addr = BASE + 4 * (len(words) - 1 - rvasm.size_of(text))
            for i in range(rvasm.size_of(text)):
                w = words[(addr - BASE) // 4 + i]
                print(f"  [{addr + 4 * i:05x}] {w:08x}  {disassemble(w, addr + 4 * i)}")
        except AsmError:
            print(f"  + {text.strip()}    (label not defined yet)")

    def _needs_label(self, line):
        """True if the line cannot be assembled on its own (forward reference)."""
        try:
            rvasm.encode(line, 0, {})
            return False
        except AsmError as e:
            if "unknown label" in str(e):
                return True
            raise

    def show_program(self):
        if not self.lines:
            print("  The program is empty. Type an instruction, or 'example' for a list.")
            return
        try:
            words = self.build()
        except AsmError as e:
            print(f"  cannot assemble yet: {e}")
            for n, line in enumerate(self.lines, 1):
                print(f"  {n:3d}  {line}")
            return
        pc, n = BASE, 0
        for n, line in enumerate(self.lines, 1):
            size = rvasm.size_of(line)
            if size == 0:
                print(f"  {n:3d}                    {line}")
            for i in range(size):
                w = words[(pc - BASE) // 4]
                mark = "=>" if pc == self.pc else "  "
                src = line if i == 0 else ""
                print(f"  {n:3d} {mark}[{pc:05x}] {w:08x}  {disassemble(w, pc):28s} {'# ' + src if src else ''}")
                pc += 4
        mark = "=>" if pc == self.pc else "  "
        print(f"      {mark}[{pc:05x}] {EBREAK:08x}  ebreak                       # end of program")

    # ---- execution ----------------------------------------------------------------
    def load(self):
        words = self.build()
        self.board.load(BASE, struct.pack(f"<{len(words)}I", *words))
        self.loaded = True

    def start(self):
        """Load the program and set up a fresh machine state."""
        self.load()
        self.regs = [0] * 32
        self.regs[1] = ROM_EXIT
        self.regs[2] = STACK
        self.pc = BASE

    def resume(self, timeout=10.0):
        """Run from self.pc with self.regs until the next trap; update the state."""
        self.board.write_words(TRAP_FRAME, self.regs)
        stub = restore_stub(self.pc)
        self.board.write(STUB, struct.pack(f"<{len(stub)}I", *stub))
        try:
            result = self.board.run(STUB, timeout=timeout)
        except ProtocolError:
            print("  The program did not stop within the time limit (infinite loop?). "
                  "Resetting the board.")
            self.board.reset()
            self.loaded = False
            self.regs = self.pc = None
            return None
        if result.output:
            text = result.output.decode("latin1").replace("\r", "")
            print("  output: " + text.rstrip("\n").replace("\n", "\n          "))
        if isinstance(result, Trap):
            self.regs = self.board.registers()
            self.pc = result.epc
        return result

    def finish(self, result):
        """Report how a run ended. Returns True if the program is over."""
        if result is None:
            return True
        if isinstance(result, Exit):
            print(f"  The program returned to the boot ROM with exit code {result.code}.")
            self.regs = self.pc = None
            return True
        if result.cause == 3 and result.epc == self.end_pc:
            print("  Program finished.")
            return True
        if result.cause == 3:
            return False                            # one of our breakpoints
        print(f"  {result}")
        print(f"  at [{result.epc:05x}] {self.describe(result.epc)}")
        return True

    def describe(self, addr):
        index = (addr - BASE) // 4
        if addr % 4 == 0 and 0 <= index < len(self.words):
            return disassemble(self.words[index], addr)
        return "(outside the program)"

    def word_at(self, addr):
        index = (addr - BASE) // 4
        if addr % 4 == 0 and 0 <= index < len(self.words):
            return self.words[index]
        return self.board.read_words(addr, 1)[0]

    def successors(self, pc, word):
        f = rvasm.fields(word)
        if f["op"] == 0b1101111:
            return [(pc + f["imm_j"]) & MASK]
        if f["op"] == 0b1100111:
            return [(self.regs[f["rs1"]] + f["imm_i"]) & MASK & ~1]
        if f["op"] == 0b1100011:
            return sorted({(pc + 4) & MASK, (pc + f["imm_b"]) & MASK})
        return [(pc + 4) & MASK]

    def run(self):
        if not self.lines:
            print("  Nothing to run. Type an instruction, or 'example' for a list.")
            return
        self.start()
        before = list(self.regs)
        self.finish(self.resume())
        self.show_changes(before)

    def cont(self):
        if self.pc is None:
            return self.run()
        before = list(self.regs)
        self.finish(self.resume())
        self.show_changes(before)

    def step(self, count=1):
        if not self.lines:
            print("  Nothing to step. Type an instruction, or 'example' for a list.")
            return
        if self.pc is None or not self.loaded:
            self.start()
        for _ in range(count):
            pc = self.pc
            if pc == self.end_pc:
                print("  Program finished. 'restart' to run it again.")
                return
            word = self.word_at(pc)
            targets = [t for t in self.successors(pc, word) if t % 4 == 0 and t < RAM_TOP]
            saved = {t: self.word_at(t) for t in targets}
            for t in targets:
                self.board.write_words(t, [EBREAK])
            before = list(self.regs)
            result = self.resume()
            if self.loaded:
                for t, original in saved.items():
                    self.board.write_words(t, [original])
            print(f"  [{pc:05x}] {word:08x}  {disassemble(word, pc)}")
            if result is None:
                return
            self.show_changes(before, indent="      ")
            if isinstance(result, Trap) and result.cause == 3 and result.epc in targets:
                continue
            if self.finish(result):
                return
        if self.pc is not None:
            print(f"  next: [{self.pc:05x}] {self.describe(self.pc)}")

    # ---- inspection ---------------------------------------------------------------
    def show_changes(self, before, indent="  "):
        if self.regs is None:
            return
        changed = [r for r in range(1, 32) if self.regs[r] != before[r]]
        if not changed:
            return
        for r in changed:
            print(f"{indent}{fmt_reg(r, self.regs[r])}")

    def current_regs(self):
        if self.regs is None:
            print("  No program has run yet; showing the registers saved at the last trap.")
            return self.board.registers()
        return self.regs

    def show_regs(self, everything=False):
        regs = self.current_regs()
        if everything:
            for row in range(8):
                print("  " + "   ".join(f"{ABI[r]:>4s}/x{r:<2d} {regs[r]:08x}"
                                         for r in range(row, 32, 8)))
            return
        shown = [r for r in range(1, 32) if regs[r]]
        for r in shown:
            print(f"  {fmt_reg(r, regs[r])}")
        if not shown:
            print("  All registers are zero.")

    def show_mem(self, addr, count):
        addr &= ~3
        words = self.board.read_words(addr, count)
        for i in range(0, count, 4):
            row = words[i:i + 4]
            text = "".join(chr(b) if 32 <= b < 127 else "." for w in row for b in struct.pack("<I", w))
            print(f"  {addr + 4 * i:08x}: " + " ".join(f"{w:08x}" for w in row) + f"   {text}")

    # ---- the rest of the system -------------------------------------------------------
    def exec_file(self, path):
        data = open(path, "rb").read()
        self.board.load(0, data)
        self.changed()
        print(f"  loaded {len(data)} bytes at 0, running")
        def echo(data):                     # program text only, not the ROM's exit report
            sys.stdout.write(data.split(b"\x1b")[0].decode("latin1").replace("\r", ""))
            sys.stdout.flush()

        result = self.board.run(0, timeout=120, echo=echo)
        print(f"\n  {result if isinstance(result, Trap) else f'exit code {result.code}'}")

    def monitor(self):
        ser = self.board.ser
        print("  Resetting into the monitor. Type its commands; '~.' returns here.")
        ser.send_break(0.05)
        ser.timeout = 0.05
        text, deadline = b"", time.time() + 4
        while not text.endswith(b"> ") and time.time() < deadline:
            text += ser.read(256)
        sys.stdout.write(text.decode("latin1").replace("\r", ""))
        sys.stdout.flush()
        if not text.endswith(b"> "):
            print("\n  The monitor did not start. Store it in flash with 'make flash-monitor'.")
            self.board.reset()
            self.changed()
            return
        stop = threading.Event()

        def pump():
            while not stop.is_set():
                data = ser.read(256)
                if data:
                    sys.stdout.write(data.decode("latin1").replace("\r", ""))
                    sys.stdout.flush()

        thread = threading.Thread(target=pump, daemon=True)
        thread.start()
        try:
            while True:
                line = input()
                if line.strip() == "~.":
                    break
                ser.write(line.encode() + b"\r")
        except (EOFError, KeyboardInterrupt):
            pass
        time.sleep(0.3)
        stop.set()
        thread.join()
        self.board.reset()
        self.changed()
        print("\n  Back in the console.")


def fmt_reg(r, value):
    signed = value - (1 << 32) if value >> 31 else value
    extra = f" ({signed})" if signed < 0 else ""
    return f"{ABI[r]:>4s}/x{r:<2d} = 0x{value:08x}  {value}{extra}"


def num(text):
    return int(text, 0)


def command(con, line):
    """Execute one console line. Returns False to quit."""
    parts = line.split(None, 1)
    cmd, args = parts[0].lower(), (parts[1].strip() if len(parts) > 1 else "")

    if cmd in ("quit", "q", "exit"):
        return False
    if cmd in ("help", "?"):
        print(HELP)
    elif cmd == "asm":
        for w in rvasm.encode(args, BASE):
            print(f"  {w:08x}  {w:032b}  {disassemble(w, BASE)}")
    elif cmd in ("prog", "list", "l"):
        con.show_program()
    elif cmd == "clear":
        con.lines = []
        con.changed()
        print("  Program cleared.")
    elif cmd == "del":
        if con.lines:
            index = num(args) - 1 if args else len(con.lines) - 1
            print(f"  removed: {con.lines.pop(index)}")
            con.changed()
    elif cmd == "example":
        if args not in EXAMPLES:
            print("  Examples:")
            for name, (what, _) in EXAMPLES.items():
                print(f"    {name:14s} {what}")
            print("  Load one with: example <name>, then 'run' or 'step'.")
        else:
            con.lines = [ln.strip() for ln in EXAMPLES[args][1].strip().splitlines()]
            con.changed()
            print(f"  {EXAMPLES[args][0]}")
            con.show_program()
    elif cmd == "source":
        con.lines = [ln.strip() for ln in open(args) if ln.split("#")[0].strip()]
        con.changed()
        con.show_program()
    elif cmd == "run":
        con.run()
    elif cmd in ("step", "s"):
        con.step(num(args) if args else 1)
    elif cmd in ("cont", "c"):
        con.cont()
    elif cmd == "restart":
        con.changed()
        print("  Back at the start.")
    elif cmd == "regs":
        con.show_regs()
    elif cmd == "dump":
        con.show_regs(everything=True)
    elif cmd == "reg":
        if not args:
            con.show_regs(everything=True)
        else:
            r = rvasm.reg(args)
            print(f"  {fmt_reg(r, con.current_regs()[r])}")
    elif cmd == "pc":
        print("  No program in progress." if con.pc is None
              else f"  pc = 0x{con.pc:08x}  {con.describe(con.pc)}")
    elif cmd == "mem":
        a = args.split()
        con.show_mem(num(a[0]), num(a[1]) if len(a) > 1 else 16)
    elif cmd == "write":
        a = args.split()
        con.board.write_words(num(a[0]) & ~3, [num(a[1])])
        print(f"  [{num(a[0]) & ~3:08x}] = {num(a[1]) & MASK:08x}")
    elif cmd == "exec":
        con.exec_file(args)
    elif cmd == "monitor":
        con.monitor()
    elif cmd == "reset":
        con.board.reset()
        con.changed()
        print("  Board reset; the boot ROM loader is ready.")
    elif cmd == "load":                     # the old console's way of adding a line
        con.add(args)
    else:
        con.add(line)                       # anything else is assembly
    return True


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB1"
    board = Board(port)
    board.reset()
    clk, ident = board.read_words(0x8000_2008, 2)
    print(f"Connected to the RV32IM SoC on {port}: {clk / 1e6:.2f} MHz, "
          f"id {struct.pack('>I', ident).decode()}")
    print("Type RISC-V assembly to build a program, then 'run' or 'step'. 'help' lists "
          "everything,\n'example' shows ready-made programs.")
    con = Console(board)
    while True:
        try:
            line = input("\nrv32> ").strip()
        except EOFError:
            break
        except KeyboardInterrupt:
            print("\n  ('quit' to leave)")
            continue
        if not line:
            continue
        try:
            if not command(con, line):
                break
        except AsmError as e:
            print(f"  {e}")
        except ProtocolError as e:
            print(f"  Lost contact with the board ({e}); resetting it.")
            try:
                board.reset()
                con.changed()
            except ProtocolError as e2:
                print(f"  {e2}")
        except (ValueError, IndexError, OSError) as e:
            print(f"  {e}")
    board.close()


if __name__ == "__main__":
    main()
