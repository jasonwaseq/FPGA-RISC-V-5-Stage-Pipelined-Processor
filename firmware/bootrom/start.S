# =============================================================================
# Boot ROM entry points (ROM base 0x0010_0000)
#
#   +0x0  reset        cold start
#   +0x4  exit         a program jumps (or returns) here with its exit code in a0
#   +0x8  trap         default trap vector (mtvec after reset)
#
# The default trap handler is the system's crash reporter: it saves every
# register in the trap frame at the top of RAM, where the host can read them
# back, reports mcause / mepc / mtval and returns to the loader.
# =============================================================================

#include "../common/soc.h"

    .section .text.start
    .globl _reset
_reset:
    j       cold_start
    j       exit_entry

    # ---- default trap vector -------------------------------------------------
trap_entry:
    csrw    mscratch, x31
    lui     x31, %hi(RAM_TOP)               # frame is the last 128 bytes of RAM
    sw      x1,  -124(x31)
    sw      x2,  -120(x31)
    sw      x3,  -116(x31)
    sw      x4,  -112(x31)
    sw      x5,  -108(x31)
    sw      x6,  -104(x31)
    sw      x7,  -100(x31)
    sw      x8,   -96(x31)
    sw      x9,   -92(x31)
    sw      x10,  -88(x31)
    sw      x11,  -84(x31)
    sw      x12,  -80(x31)
    sw      x13,  -76(x31)
    sw      x14,  -72(x31)
    sw      x15,  -68(x31)
    sw      x16,  -64(x31)
    sw      x17,  -60(x31)
    sw      x18,  -56(x31)
    sw      x19,  -52(x31)
    sw      x20,  -48(x31)
    sw      x21,  -44(x31)
    sw      x22,  -40(x31)
    sw      x23,  -36(x31)
    sw      x24,  -32(x31)
    sw      x25,  -28(x31)
    sw      x26,  -24(x31)
    sw      x27,  -20(x31)
    sw      x28,  -16(x31)
    sw      x29,  -12(x31)
    sw      x30,   -8(x31)
    csrr    x1, mscratch
    sw      x1,    -4(x31)
    jal     ra, rom_init
    j       trap_report                     # does not return

exit_entry:
    mv      s0, a0
    jal     ra, rom_init
    mv      a0, s0
    j       exit_report                     # does not return

cold_start:
    jal     ra, rom_init
    j       boot_main                       # does not return

    # Put the machine back in a known state: interrupts off, default trap
    # vector, ROM stack.
rom_init:
    csrci   mstatus, 8
    csrw    mie, zero
    la      t0, trap_entry
    csrw    mtvec, t0
    li      sp, ROM_STACK_TOP
    ret

    # void run(uint32_t entry): start a program. Its return address is the
    # exit entry, so `return n` from main and exit(n) end up in the same place.
    # FENCE.I first: the code may just have been written to RAM.
    .globl run
run:
    fence.i
    mv      t0, a0
    la      ra, _reset + 4
    li      a0, 0
    jr      t0
