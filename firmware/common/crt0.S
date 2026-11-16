# C runtime start-up, shared by programs that run from RAM and from flash.
#include "soc.h"

    .section .text.start
    .globl _start
_start:
    .option push
    .option norelax
    la      gp, __global_pointer$
    .option pop
    la      sp, __stack_top

    # Copy initialised data out of flash (execute-in-place images only; for a
    # RAM image the load and run addresses are the same and nothing moves).
    la      a0, __data_load
    la      a1, __data_start
    la      a2, __data_end
    beq     a0, a1, 2f
1:  bgeu    a1, a2, 2f
    lw      t0, 0(a0)
    sw      t0, 0(a1)
    addi    a0, a0, 4
    addi    a1, a1, 4
    j       1b

2:  la      a1, __bss_start
    la      a2, __bss_end
3:  bgeu    a1, a2, 4f
    sw      zero, 0(a1)
    addi    a1, a1, 4
    j       3b

4:  call    main
    # fall through: exit(main())

    .globl exit
exit:
    li      t0, ROM_EXIT
    jr      t0
