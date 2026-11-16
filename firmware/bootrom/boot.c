/*
 * First-stage boot loader, in the 2 KB boot ROM.
 *
 * After reset it listens on the UART for half a second. If the host sends '!'
 * it becomes a memory loader; otherwise it boots the image stored in SPI
 * flash, and falls back to the loader if there is none.
 *
 * Loader commands (binary, little endian):
 *   '!'                  -> 'K'                 ping
 *   'W' addr len <data>  -> 'K'                 write memory
 *   'R' addr len         -> <data>              read memory
 *   'C' addr len         -> crc32               CRC-32 of a memory range
 *   'G' addr             -> 'K'                 FENCE.I and jump
 *
 * A program ends by returning from its entry point or jumping to ROM_EXIT;
 * a trap it does not handle itself lands in the ROM's trap vector. Both are
 * reported to the host (ESC 'X' code / ESC 'T' mcause mepc mtval) and the ROM
 * goes back to the loader, so the host never loses the board.
 */
#include "../common/soc.h"

void run(uint32_t entry) __attribute__((noreturn));
static void loader(void) __attribute__((noreturn));

static void putch(int c)
{
    while (!(REG32(UART_STATUS) & UART_TX_READY))
        ;
    REG32(UART_DATA) = c;
}

static int getch(void)
{
    int32_t v;
    do
        v = REG32(UART_DATA);
    while (v < 0);
    return v & 0xFF;
}

static void print(const char *s)
{
    while (*s)
        putch(*s++);
}

static uint32_t get32(void)
{
    uint32_t v = 0;
    for (int i = 0; i < 32; i += 8)
        v |= (uint32_t)getch() << i;
    return v;
}

static void put32(uint32_t v)
{
    for (int i = 0; i < 4; i++, v >>= 8)
        putch(v & 0xFF);
}

static uint32_t crc32(const uint8_t *p, uint32_t n)
{
    uint32_t crc = 0xFFFFFFFF;
    while (n--) {
        crc ^= *p++;
        for (int i = 0; i < 8; i++)
            crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
    }
    return ~crc;
}

static void loader(void)
{
    for (;;) {
        int cmd = getch();
        if (cmd == '!') {
            putch('K');
            continue;
        }
        if (cmd != 'W' && cmd != 'R' && cmd != 'C' && cmd != 'G') {
            putch('?');
            continue;
        }
        uint8_t *p = (uint8_t *)get32();
        if (cmd == 'G') {
            putch('K');
            run((uint32_t)p);
        }
        uint32_t n = get32();
        if (cmd == 'C') {
            put32(crc32(p, n));
        } else if (cmd == 'W') {
            while (n--)
                *p++ = getch();
            putch('K');
        } else {
            while (n--)
                putch(*p++);
        }
    }
}

static void report(int kind)
{
    putch(REPORT_ESC);
    putch(kind);
}

void exit_report(uint32_t code)
{
    report(REPORT_EXIT);
    put32(code);
    loader();
}

void trap_report(void)
{
    uint32_t cause, epc, tval;
    __asm__ volatile("csrr %0, mcause" : "=r"(cause));
    __asm__ volatile("csrr %0, mepc" : "=r"(epc));
    __asm__ volatile("csrr %0, mtval" : "=r"(tval));
    REG32(GPIO_OUT) = LED_RED;
    report(REPORT_TRAP);
    put32(cause);
    put32(epc);
    put32(tval);
    loader();
}

void boot_main(void)
{
    REG32(GPIO_OUT) = LED_RED;
    print("\r\nRV32IM SoC boot ROM\r\n");

    /* Give the host half a second to claim the board. */
    uint32_t start = REG32(MTIME_LO);
    uint32_t wait = REG32(SYS_CLK_HZ) >> 1;
    while (REG32(MTIME_LO) - start < wait) {
        if (REG32(UART_DATA) == '!') {
            putch('K');
            loader();
        }
    }

    const struct boot_header *h = (const struct boot_header *)BOOT_IMAGE;
    if (h->magic == BOOT_MAGIC) {
        const uint32_t *src = (const uint32_t *)(h + 1);
        uint32_t n = h->size;                   /* a multiple of 4 */
        if (h->load_addr != BOOT_XIP) {         /* copy to RAM, check there */
            uint32_t *dst = (uint32_t *)h->load_addr;
            for (uint32_t i = 0; i < n / 4; i++)
                dst[i] = src[i];
            src = dst;
        }
        if (crc32((const uint8_t *)src, n) == h->crc) {
            REG32(GPIO_OUT) = LED_GREEN;
            run(h->entry);
        }
        print("boot image corrupt\r\n");
    } else {
        print("no boot image\r\n");
    }
    loader();
}
