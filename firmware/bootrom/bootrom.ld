/* Boot ROM: 2 KB of code and constants, no writable data (stack only). */
OUTPUT_ARCH(riscv)
ENTRY(_reset)

MEMORY
{
    ROM (rx) : ORIGIN = 0x00100000, LENGTH = 2048
}

SECTIONS
{
    .text : {
        KEEP(*(.text.start))
        *(.text .text.*)
        *(.rodata .rodata.* .srodata .srodata.*)
    } > ROM

    .data : { *(.data .data.* .sdata .sdata.*) } > ROM
    .bss  : { *(.bss .bss.* .sbss .sbss.* COMMON) } > ROM

    /DISCARD/ : { *(.comment) *(.riscv.attributes) *(.eh_frame) }
}

ASSERT(SIZEOF(.data) == 0, "the boot ROM cannot have initialised data")
ASSERT(SIZEOF(.bss) == 0, "the boot ROM cannot have static variables")
