/*
 * Interrupt test: machine timer, software and external (UART receive)
 * interrupts, taken in the middle of real work and returned from.
 *
 * Exit code 0 = pass, otherwise the number of the check that failed.
 */
#include "lib.h"

static volatile uint32_t ticks, soft_hits, rx_count;
static volatile char rx_buf[16];
static uint32_t tick_period;

void irq_handler(uint32_t cause)
{
    switch (cause & 0xFF) {
    case IRQ_TIMER: {
        ticks++;
        uint64_t next = rdcycle64() + tick_period;
        REG32(MTIMECMP_HI) = 0xFFFFFFFF;        /* no spurious match while updating */
        REG32(MTIMECMP_LO) = (uint32_t)next;
        REG32(MTIMECMP_HI) = (uint32_t)(next >> 32);
        break;
    }
    case IRQ_SOFT:
        soft_hits++;
        REG32(MSIP) = 0;
        break;
    case IRQ_EXT:
        for (int c; (c = getchar_nb()) >= 0;)
            if (rx_count < sizeof rx_buf)
                rx_buf[rx_count++] = c;
        break;
    }
}

/* Work whose result is wrong if an interrupt corrupts any register. The
 * length is volatile so the compiler cannot fold the whole thing away. */
static volatile uint32_t work = 3000;

static __attribute__((noinline)) uint32_t checksum(uint32_t n)
{
    uint32_t a = 1, b = 0;
    for (uint32_t i = 0; i < n; i++) {
        a = a * 1664525u + 1013904223u;
        b += (a >> 7) ^ (b << 3) ^ i;
        if (a & 0x100)
            b = b / 3 + (a % 7);
    }
    return b;
}

int main(void)
{
    tick_period = clk_hz() / 200;
    irq_init();

    /* 1: the reference value, computed with interrupts off */
    uint32_t want = checksum(work);

    /* 2: timer interrupts while the same work runs */
    uint64_t now = rdcycle64() + tick_period;
    REG32(MTIMECMP_LO) = (uint32_t)now;
    REG32(MTIMECMP_HI) = (uint32_t)(now >> 32);
    csr_set(mie, MIE_MTIE);
    csr_set(mstatus, MSTATUS_MIE);
    for (int round = 0; round < 400 && ticks < 5; round++)
        if (checksum(work) != want)
            return 2;
    uint32_t seen = ticks;
    if (seen < 5)
        return 3;

    /* 4: software interrupt, taken immediately */
    csr_set(mie, MIE_MSIE);
    REG32(MSIP) = 1;
    for (volatile int i = 0; i < 20 && !soft_hits; i++)
        ;
    if (soft_hits != 1)
        return 4;

    /* 5: with MIE clear nothing is delivered, though it stays pending */
    csr_clear(mstatus, MSTATUS_MIE);
    uint32_t before = ticks;
    REG32(MSIP) = 1;
    for (volatile int i = 0; i < 2000; i++)
        ;
    if (soft_hits != 1 || ticks != before || !(csr_read(mip) & MIE_MSIE))
        return 5;
    csr_set(mstatus, MSTATUS_MIE);
    for (volatile int i = 0; i < 20 && soft_hits != 2; i++)
        ;
    if (soft_hits != 2)
        return 6;

    /* 7: UART receive interrupt; the host sends "hello" when asked */
    csr_set(mie, MIE_MEIE);
    REG32(UART_CTRL) = UART_IRQ_RX;
    printf("ticks=%u, send\n", (unsigned)seen);
    uint32_t t0 = ticks;
    while (rx_count < 5 && ticks - t0 < 4000)
        ;
    if (rx_count != 5 || memcmp((const char *)rx_buf, "hello", 5))
        return 7;

    csr_clear(mstatus, MSTATUS_MIE);
    REG32(UART_CTRL) = 0;
    printf("interrupts ok: %u timer, %u software, %u uart\n",
           (unsigned)ticks, (unsigned)soft_hits, (unsigned)rx_count);
    return 0;
}
