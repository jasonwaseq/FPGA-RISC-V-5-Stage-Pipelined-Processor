/* Memory map and register definitions of the SoC. Usable from C and assembly. */
#ifndef SOC_H
#define SOC_H

#define RAM_BASE        0x00000000
#define RAM_TOP         0x00020000      /* 128 KB */
#define ROM_BASE        0x00100000
#define FLASH_BASE      0x01000000      /* 16 MB, read-only */
#define MMIO_BASE       0x80000000

/* Top of RAM, reserved for the boot ROM */
#define TRAP_FRAME      (RAM_TOP - 128) /* x0..x31 saved by the ROM trap handler */
#define ROM_STACK_TOP   TRAP_FRAME
#define APP_STACK_TOP   (RAM_TOP - 512)

/* ROM entry points */
#define ROM_RESET       (ROM_BASE + 0)
#define ROM_EXIT        (ROM_BASE + 4)  /* a0 = exit code */
#define ROM_TRAP        (ROM_BASE + 8)

/* Boot image in flash: the bitstream owns the first megabyte */
#define BOOT_IMAGE      (FLASH_BASE + 0x100000)
#define BOOT_MAGIC      0x42355652      /* "RV5B" */

#define UART_DATA       (MMIO_BASE + 0x0000)    /* r: byte, bit 31 = empty; w: send */
#define UART_STATUS     (MMIO_BASE + 0x0004)
#define UART_CTRL       (MMIO_BASE + 0x0008)
#define UART_RX_VALID   1
#define UART_TX_READY   2
#define UART_OVERRUN    4
#define UART_IRQ_RX     1
#define UART_IRQ_TX     2

#define MTIME_LO        (MMIO_BASE + 0x1000)
#define MTIME_HI        (MMIO_BASE + 0x1004)
#define MTIMECMP_LO     (MMIO_BASE + 0x1008)
#define MTIMECMP_HI     (MMIO_BASE + 0x100C)
#define MSIP            (MMIO_BASE + 0x1010)

#define GPIO_OUT        (MMIO_BASE + 0x2000)
#define GPIO_IN         (MMIO_BASE + 0x2004)
#define SYS_CLK_HZ      (MMIO_BASE + 0x2008)
#define SYS_ID          (MMIO_BASE + 0x200C)

#define LED_RED         0x01
#define LED_GREEN       0x02

/* Framing of the reports the ROM sends when a program ends */
#define REPORT_ESC      0x1B
#define REPORT_EXIT     'X'             /* + exit code (4 bytes, little endian) */
#define REPORT_TRAP     'T'             /* + mcause, mepc, mtval */

#ifndef __ASSEMBLER__
#include <stdint.h>
#define REG32(a)        (*(volatile uint32_t *)(a))

/* Header of a boot image, followed by `size` bytes of payload. If load_addr
 * is 0xFFFFFFFF the payload is executed in place from flash. */
struct boot_header {
    uint32_t magic;
    uint32_t load_addr;
    uint32_t size;
    uint32_t entry;
    uint32_t crc;
};
#define BOOT_XIP        0xFFFFFFFFu
#endif

#endif
