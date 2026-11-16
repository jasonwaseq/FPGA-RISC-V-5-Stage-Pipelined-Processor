/* Program executed in place from SPI flash: code and constants stay in flash
 * (behind the 20-byte boot header at the 1 MB mark), data lives in RAM. */
OUTPUT_ARCH(riscv)
ENTRY(_start)

MEMORY
{
    FLASH (rx) : ORIGIN = 0x01100014, LENGTH = 0x100000
    RAM  (rwx) : ORIGIN = 0x00010000, LENGTH = 0x10000 - 512  /* upper half of RAM */
}

SECTIONS
{
    .text : {
        KEEP(*(.text.start))
        *(.text .text.*)
    } > FLASH

    .rodata : ALIGN(4) {
        *(.rodata .rodata.* .srodata .srodata.*)
        . = ALIGN(4);
    } > FLASH

    .data : ALIGN(4) {
        __data_start = .;
        __global_pointer$ = . + 0x800;
        *(.sdata .sdata.* .data .data.*)
        . = ALIGN(4);
        __data_end = .;
    } > RAM AT > FLASH
    __data_load = LOADADDR(.data);
    __image_end = __data_load + SIZEOF(.data);

    .bss (NOLOAD) : ALIGN(4) {
        __bss_start = .;
        *(.sbss .sbss.* .bss .bss.* COMMON)
        . = ALIGN(4);
        __bss_end = .;
    } > RAM

    __heap_start = .;
    __stack_top = ORIGIN(RAM) + LENGTH(RAM);

    /DISCARD/ : { *(.comment) *(.riscv.attributes) *(.eh_frame) }
}
