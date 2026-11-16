/*
 * Monitor: the interactive shell of the SoC.
 *
 * It is stored in SPI flash behind the bitstream and executed in place: the
 * boot ROM finds its header at power-up and jumps into flash, the I-cache
 * pulls the code in line by line, and only its data lives in RAM (the upper
 * 64 KB; the lower 64 KB is free for whatever the user loads).
 *
 * A 100 Hz timer interrupt keeps the uptime and blinks an LED.
 */
#include "lib.h"

#define WORK        ((uint32_t *)0x4000)        /* scratch area for benchmarks */
#define WORK_BYTES  0x8000

static volatile uint32_t ticks;
static uint32_t tick_period;

void irq_handler(uint32_t cause)
{
    if ((cause & 0xFF) != IRQ_TIMER)
        return;
    ticks++;
    if (ticks % 50 == 0)
        REG32(GPIO_OUT) ^= LED_1;
    uint64_t next = rdcycle64() + tick_period;
    REG32(MTIMECMP_HI) = 0xFFFFFFFF;
    REG32(MTIMECMP_LO) = (uint32_t)next;
    REG32(MTIMECMP_HI) = (uint32_t)(next >> 32);
}

/* ---- Performance counters ------------------------------------------------------ */
struct perf {
    uint64_t cycles, instrs;
    uint32_t icmiss, mispred, branches, stalls;
};

static void perf_read(struct perf *p)
{
    p->cycles = rdcycle64();
    p->instrs = rdinstret64();
    p->icmiss = csr_read(mhpmcounter3);
    p->mispred = csr_read(mhpmcounter4);
    p->branches = csr_read(mhpmcounter5);
    p->stalls = csr_read(mhpmcounter6);
}

static void perf_print(const char *name, const struct perf *a, const struct perf *b)
{
    uint32_t cycles = (uint32_t)(b->cycles - a->cycles);
    uint32_t instrs = (uint32_t)(b->instrs - a->instrs);
    uint32_t cpi = instrs ? (uint32_t)((uint64_t)cycles * 1000 / instrs) : 0;
    uint32_t br = b->branches - a->branches, mp = b->mispred - a->mispred;
    uint32_t rate = br ? (uint32_t)((uint64_t)mp * 1000 / br) : 0;
    printf("%-10s %9u cycles %9u instrs  CPI %u.%03u  I-miss %5u  mispredict %6u (%u.%u%% of %u branches)  interlock %u\n",
           name, (unsigned)cycles, (unsigned)instrs, (unsigned)(cpi / 1000), (unsigned)(cpi % 1000),
           (unsigned)(b->icmiss - a->icmiss), (unsigned)mp, (unsigned)(rate / 10),
           (unsigned)(rate % 10), (unsigned)br, (unsigned)(b->stalls - a->stalls));
}

/* ---- Built-in benchmarks --------------------------------------------------------- */
static uint32_t lfsr(uint32_t x)
{
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    return x;
}

static uint32_t bench_sieve(void)
{
    uint8_t *s = (uint8_t *)WORK;
    uint32_t n = 20000, count = 0;
    memset(s, 1, n);
    for (uint32_t i = 2; i < n; i++) {
        if (!s[i])
            continue;
        count++;
        for (uint32_t j = i * i; j < n; j += i)
            s[j] = 0;
    }
    return count;                               /* 2262 */
}

static uint32_t bench_sort(void)
{
    uint32_t *a = WORK, n = 1500, x = 12345;
    for (uint32_t i = 0; i < n; i++)
        a[i] = x = lfsr(x);
    for (uint32_t gap = n / 2; gap; gap /= 2)   /* shell sort */
        for (uint32_t i = gap; i < n; i++) {
            uint32_t v = a[i], j = i;
            for (; j >= gap && a[j - gap] > v; j -= gap)
                a[j] = a[j - gap];
            a[j] = v;
        }
    uint32_t ok = 1;
    for (uint32_t i = 1; i < n; i++)
        ok &= a[i - 1] <= a[i];
    return ok;
}

static uint32_t bench_crc(void)
{
    uint32_t x = 99;
    for (uint32_t i = 0; i < 2048; i++)
        WORK[i] = x = lfsr(x);
    return crc32(WORK, 8192);
}

static uint32_t bench_matmul(void)
{
    enum { N = 20 };
    int32_t *a = (int32_t *)WORK, *b = a + N * N, *c = b + N * N;
    for (int i = 0; i < N * N; i++) {
        a[i] = i % 17 - 8;
        b[i] = i % 13 - 6;
    }
    int32_t trace = 0;
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++) {
            int32_t sum = 0;
            for (int k = 0; k < N; k++)
                sum += a[i * N + k] * b[k * N + j];
            c[i * N + j] = sum;
            if (i == j)
                trace += sum;
        }
    return (uint32_t)trace;
}

static uint32_t bench_divide(void)
{
    uint32_t x = 7, acc = 0;
    for (uint32_t i = 1; i <= 500; i++) {
        x = lfsr(x);
        acc += x / i + x % (i + 3);
    }
    return acc;
}

static void cmd_bench(void)
{
    static const struct {
        const char *name;
        uint32_t (*fn)(void);
    } list[] = {
        {"sieve", bench_sieve}, {"shellsort", bench_sort}, {"crc32", bench_crc},
        {"matmul", bench_matmul}, {"divide", bench_divide},
    };
    for (unsigned i = 0; i < sizeof list / sizeof list[0]; i++) {
        struct perf a, b;
        perf_read(&a);
        uint32_t r = list[i].fn();
        perf_read(&b);
        perf_print(list[i].name, &a, &b);
        printf("           result %08x\n", (unsigned)r);
    }
}

/* ---- Commands ------------------------------------------------------------------- */
static void cmd_info(void)
{
    uint32_t hz = clk_hz(), s = ticks / 100;
    printf("RV32IM_Zicsr_Zifencei, 5-stage pipeline, M-mode\n");
    printf("clock   %u.%02u MHz\n", (unsigned)(hz / 1000000), (unsigned)(hz / 10000 % 100));
    printf("memory  128 KB RAM at 0x00000000, boot ROM at 0x%08x, 16 MB flash at 0x%08x\n",
           ROM_BASE, FLASH_BASE);
    printf("cache   8 KB direct-mapped I-cache (32-byte lines), 256-entry BTB\n");
    printf("misa    %08x   id %08x\n", (unsigned)csr_read(misa), (unsigned)REG32(SYS_ID));
    printf("uptime  %u:%02u:%02u (%u timer interrupts)\n", (unsigned)(s / 3600),
           (unsigned)(s / 60 % 60), (unsigned)(s % 60), (unsigned)ticks);
}

static void cmd_md(uint32_t addr, uint32_t words)
{
    addr &= ~3u;
    for (uint32_t i = 0; i < words; i++) {
        if (i % 4 == 0)
            printf("%08x:", (unsigned)(addr + 4 * i));
        printf(" %08x", (unsigned)REG32(addr + 4 * i));
        if (i % 4 == 3 || i == words - 1)
            printf("\n");
    }
}

static void cmd_load(uint32_t addr, uint32_t len)
{
    uint8_t *p = (uint8_t *)addr;
    printf("ready\n");
    for (uint32_t i = 0; i < len; i++)
        p[i] = getchar_();
    printf("loaded %u bytes at %08x, crc %08x\n", (unsigned)len, (unsigned)addr,
           (unsigned)crc32(p, len));
}

static void cmd_go(uint32_t addr)
{
    __asm__ volatile("fence.i");
    uint32_t r = ((uint32_t (*)(void))addr)();
    printf("returned %u (0x%x)\n", (unsigned)r, (unsigned)r);
}

static void cmd_help(void)
{
    puts_("info                 system summary and uptime\n"
          "md <addr> [words]    dump memory\n"
          "mw <addr> <value>    write a word\n"
          "load <addr> <len>    receive <len> raw bytes into memory\n"
          "crc <addr> <len>     CRC-32 of a memory range\n"
          "go <addr>            call a loaded program\n"
          "perf                 performance counters since reset\n"
          "bench                built-in benchmarks with counter deltas\n"
          "led <value>          set the LEDs\n"
          "reboot               back to the boot ROM\n");
}

static int split(char *line, char **argv, int max)
{
    int argc = 0;
    while (*line && argc < max) {
        while (*line == ' ')
            line++;
        if (!*line)
            break;
        argv[argc++] = line;
        while (*line && *line != ' ')
            line++;
        if (*line)
            *line++ = 0;
    }
    return argc;
}

int main(void)
{
    tick_period = clk_hz() / 100;
    irq_init();
    uint64_t first = rdcycle64() + tick_period;
    REG32(MTIMECMP_LO) = (uint32_t)first;
    REG32(MTIMECMP_HI) = (uint32_t)(first >> 32);
    csr_set(mie, MIE_MTIE);
    csr_set(mstatus, MSTATUS_MIE);
    REG32(GPIO_OUT) = LED_1;

    printf("\nRV32IM monitor (running in place from SPI flash). Type 'help'.\n");

    char line[80], *argv[4];
    for (;;) {
        puts_("> ");
        readline(line, sizeof line);
        int argc = split(line, argv, 4), ok1 = 0, ok2 = 0;
        if (!argc)
            continue;
        uint32_t a = argc > 1 ? parse_num(argv[1], &ok1) : 0;
        uint32_t b = argc > 2 ? parse_num(argv[2], &ok2) : 0;
        const char *cmd = argv[0];

        if (!strcmp(cmd, "help"))
            cmd_help();
        else if (!strcmp(cmd, "info"))
            cmd_info();
        else if (!strcmp(cmd, "md") && ok1)
            cmd_md(a, ok2 ? b : 16);
        else if (!strcmp(cmd, "mw") && ok1 && ok2)
            REG32(a & ~3u) = b;
        else if (!strcmp(cmd, "load") && ok1 && ok2)
            cmd_load(a, b);
        else if (!strcmp(cmd, "crc") && ok1 && ok2)
            printf("%08x\n", (unsigned)crc32((void *)a, b));
        else if (!strcmp(cmd, "go") && ok1)
            cmd_go(a);
        else if (!strcmp(cmd, "perf")) {
            static struct perf last;
            struct perf now;
            perf_read(&now);
            perf_print("interval", &last, &now);
            last = now;
        } else if (!strcmp(cmd, "bench"))
            cmd_bench();
        else if (!strcmp(cmd, "led") && ok1)
            REG32(GPIO_OUT) = a;
        else if (!strcmp(cmd, "reboot"))
            ((void (*)(void))ROM_RESET)();
        else
            printf("unknown command or bad arguments: %s\n", cmd);
    }
}
