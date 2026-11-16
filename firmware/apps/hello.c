/* Smallest useful program: prints, measures itself, exits with a code. */
#include "lib.h"

int main(void)
{
    uint64_t c0 = rdcycle64(), i0 = rdinstret64();
    printf("Hello from RV32IM at %u Hz\n", clk_hz());

    uint32_t sum = 0;
    for (uint32_t i = 1; i <= 1000; i++)
        sum += i * i;

    uint32_t cycles = (uint32_t)(rdcycle64() - c0);
    uint32_t instrs = (uint32_t)(rdinstret64() - i0);
    printf("sum of squares 1..1000 = %u\n", sum);
    printf("%u instructions in %u cycles\n", instrs, cycles);
    return sum == 333833500 ? 42 : 1;
}
