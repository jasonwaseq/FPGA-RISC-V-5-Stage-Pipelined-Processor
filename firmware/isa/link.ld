/* riscv-tests on this SoC: the stock "p" environment, linked at the RAM base. */
OUTPUT_ARCH("riscv")
ENTRY(_start)

SECTIONS
{
    . = 0x00000000;
    .text.init : { *(.text.init) }
    . = ALIGN(0x100);
    .tohost : { *(.tohost) }
    . = ALIGN(0x100);
    .text : { *(.text) }
    . = ALIGN(0x100);
    .data : { *(.data) *(.data.*) }
    .bss : { *(.bss) }
    _end = .;

    /DISCARD/ : { *(.comment) *(.riscv.attributes) }
}
