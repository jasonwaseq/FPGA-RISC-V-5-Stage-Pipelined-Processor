/*
 * Memory test for the 128 KB SPRAM: every word above this program, every
 * access width, and the paths the pipeline takes for them (store followed by
 * load, byte and halfword lanes, both RAM banks).
 *
 * Exit code 0 = pass, otherwise the number of the failing pass.
 */
#include "lib.h"

extern char __heap_start[];

static uint32_t lfsr(uint32_t x)
{
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    return x;
}

int main(void)
{
    uint32_t *lo = (uint32_t *)(((uintptr_t)__heap_start + 3) & ~3u);
    uint32_t *hi = (uint32_t *)(APP_STACK_TOP - 1024);      /* keep clear of the stack */
    uint32_t words = hi - lo;
    printf("memtest: %u KB from %p to %p\n", (unsigned)(words / 256), (void *)lo, (void *)hi);

    /* 1: address in address */
    for (uint32_t *p = lo; p < hi; p++)
        *p = (uint32_t)p;
    for (uint32_t *p = lo; p < hi; p++)
        if (*p != (uint32_t)p)
            return 1;

    /* 2: pseudo-random data and its complement */
    for (int pass = 0; pass < 2; pass++) {
        uint32_t x = 0x1234ABCD;
        for (uint32_t *p = lo; p < hi; p++) {
            x = lfsr(x);
            *p = pass ? ~x : x;
        }
        x = 0x1234ABCD;
        for (uint32_t *p = lo; p < hi; p++) {
            x = lfsr(x);
            if (*p != (pass ? ~x : x))
                return 2;
        }
    }

    /* 3: walking ones, read back straight after each write */
    for (uint32_t *p = lo; p < hi; p += 251)
        for (uint32_t bit = 1; bit; bit <<= 1) {
            *p = bit;
            if (*p != bit)
                return 3;
        }

    /* 4: byte and halfword lanes */
    uint8_t *b = (uint8_t *)lo;
    uint32_t bytes = words * 4;
    for (uint32_t i = 0; i < bytes; i++)
        b[i] = (uint8_t)(i * 7 + (i >> 8));
    for (uint32_t i = 0; i < bytes; i++)
        if (b[i] != (uint8_t)(i * 7 + (i >> 8)))
            return 4;
    uint16_t *h = (uint16_t *)lo;
    for (uint32_t i = 0; i < bytes / 2; i += 3)
        h[i] = (uint16_t)(i ^ 0xA5C3);
    for (uint32_t i = 0; i < bytes / 2; i += 3)
        if (h[i] != (uint16_t)(i ^ 0xA5C3))
            return 5;

    /* 6: memcpy across the bank boundary at 64 KB */
    uint32_t *mid = (uint32_t *)0x10000;
    if (lo < mid - 512 && hi > mid + 512) {
        for (int i = 0; i < 512; i++)
            mid[i - 512] = lfsr(i + 1);
        memcpy(mid - 256, mid - 512, 1024);
        memcpy(mid, mid - 256, 1024);
        for (int i = 0; i < 256; i++)
            if (mid[i] != lfsr(i + 1))
                return 6;
    }

    printf("memtest: pass\n");
    return 0;
}
