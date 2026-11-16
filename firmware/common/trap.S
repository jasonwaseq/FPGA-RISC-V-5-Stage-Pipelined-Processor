# Program-level trap entry (installed by irq_init).
#
# Interrupts: save the caller-saved registers, call irq_handler(mcause), MRET.
# Exceptions: not ours to handle. Enter the boot ROM's crash reporter exactly
# as if the trap had gone there directly, with every register still intact
# (the ROM entry is joined just after its own "csrw mscratch, x31").
#include "soc.h"

    .section .text
    .globl trap_entry
    .align 2
trap_entry:
    csrw    mscratch, t6
    csrr    t6, mcause
    bltz    t6, 1f
    lui     t6, %hi(ROM_TRAP + 4)
    jr      %lo(ROM_TRAP + 4)(t6)

1:  addi    sp, sp, -64
    sw      ra,  0(sp)
    sw      t0,  4(sp)
    sw      t1,  8(sp)
    sw      t2, 12(sp)
    sw      a0, 16(sp)
    sw      a1, 20(sp)
    sw      a2, 24(sp)
    sw      a3, 28(sp)
    sw      a4, 32(sp)
    sw      a5, 36(sp)
    sw      a6, 40(sp)
    sw      a7, 44(sp)
    sw      t3, 48(sp)
    sw      t4, 52(sp)
    sw      t5, 56(sp)
    mv      a0, t6
    call    irq_handler
    lw      ra,  0(sp)
    lw      t0,  4(sp)
    lw      t1,  8(sp)
    lw      t2, 12(sp)
    lw      a0, 16(sp)
    lw      a1, 20(sp)
    lw      a2, 24(sp)
    lw      a3, 28(sp)
    lw      a4, 32(sp)
    lw      a5, 36(sp)
    lw      a6, 40(sp)
    lw      a7, 44(sp)
    lw      t3, 48(sp)
    lw      t4, 52(sp)
    lw      t5, 56(sp)
    addi    sp, sp, 64
    csrr    t6, mscratch
    mret
