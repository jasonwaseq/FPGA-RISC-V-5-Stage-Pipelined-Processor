/* Program loaded into RAM (by the boot ROM over UART, or copied from flash). */
OUTPUT_ARCH(riscv)
ENTRY(_start)

MEMORY
{
    RAM (rwx) : ORIGIN = 0x00000000, LENGTH = 0x20000 - 512   /* top: ROM stack, trap frame */
}

SECTIONS
{
    .text : {
        KEEP(*(.text.start))
        *(.text .text.*)
    } > RAM

    .rodata : ALIGN(4) {
        *(.rodata .rodata.* .srodata .srodata.*)
    } > RAM

    .data : ALIGN(4) {
        __data_start = .;
        __global_pointer$ = . + 0x800;
        *(.sdata .sdata.* .data .data.*)
        . = ALIGN(4);
        __data_end = .;
    } > RAM
    __data_load = __data_start;
    __image_end = __data_end;

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
