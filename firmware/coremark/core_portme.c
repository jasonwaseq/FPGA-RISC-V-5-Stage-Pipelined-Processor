/*
 * CoreMark port for the RV32IM SoC.
 *
 * Time is the core's cycle counter, so the score in CoreMark/MHz does not
 * depend on the clock the bitstream was built for. The run also reports the
 * hardware performance counters for the timed section.
 */
#include <stdarg.h>
#include "coremark.h"
#include "lib.h"

#if VALIDATION_RUN
volatile ee_s32 seed1_volatile = 0x3415;
volatile ee_s32 seed2_volatile = 0x3415;
volatile ee_s32 seed3_volatile = 0x66;
#endif
#if PERFORMANCE_RUN
volatile ee_s32 seed1_volatile = 0x0;
volatile ee_s32 seed2_volatile = 0x0;
volatile ee_s32 seed3_volatile = 0x66;
#endif
#if PROFILE_RUN
volatile ee_s32 seed1_volatile = 0x8;
volatile ee_s32 seed2_volatile = 0x8;
volatile ee_s32 seed3_volatile = 0x8;
#endif
volatile ee_s32 seed4_volatile = ITERATIONS;
volatile ee_s32 seed5_volatile = 0;

ee_u32 default_num_contexts = 1;

static struct {
    uint64_t cycles, instrs;
    uint32_t icmiss, mispred, branches, stalls;
} t0, t1;

static void snapshot(void *where)
{
    __typeof__(t0) *p = where;
    p->cycles = rdcycle64();
    p->instrs = rdinstret64();
    p->icmiss = csr_read(mhpmcounter3);
    p->mispred = csr_read(mhpmcounter4);
    p->branches = csr_read(mhpmcounter5);
    p->stalls = csr_read(mhpmcounter6);
}

void start_time(void)
{
    snapshot(&t0);
}

void stop_time(void)
{
    snapshot(&t1);
}

CORE_TICKS get_time(void)
{
    return t1.cycles - t0.cycles;
}

/* CoreMark only uses this to check that the run lasted at least 10 seconds. */
secs_ret time_in_secs(CORE_TICKS ticks)
{
    return (secs_ret)(ticks / clk_hz());
}

int ee_printf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int n = vprintf_(fmt, ap);
    va_end(ap);
    return n;
}

void portable_init(core_portable *p, int *argc, char *argv[])
{
    (void)argc;
    (void)argv;
    p->portable_id = 1;
}

void portable_fini(core_portable *p)
{
    uint64_t cycles = t1.cycles - t0.cycles, instrs = t1.instrs - t0.instrs;
    uint32_t iterations = (uint32_t)seed4_volatile;
    uint32_t cpi = (uint32_t)(cycles * 1000 / instrs);
    uint32_t br = t1.branches - t0.branches, mp = t1.mispred - t0.mispred;

    printf("\n--- hardware counters for the timed run ---\n");
    printf("cycles        : %u\n", (unsigned)cycles);
    printf("instructions  : %u\n", (unsigned)instrs);
    printf("CPI           : %u.%03u\n", (unsigned)(cpi / 1000), (unsigned)(cpi % 1000));
    printf("branches      : %u, mispredicted %u (%u.%02u%%)\n", (unsigned)br, (unsigned)mp,
           (unsigned)((uint64_t)mp * 100 / br), (unsigned)((uint64_t)mp * 10000 / br % 100));
    printf("I-cache misses: %u\n", (unsigned)(t1.icmiss - t0.icmiss));
    printf("interlock     : %u stall cycles\n", (unsigned)(t1.stalls - t0.stalls));
    printf("clock         : %u Hz\n", (unsigned)clk_hz());
    if (iterations) {
        uint32_t score = (uint32_t)((uint64_t)iterations * 1000000000u / cycles);
        printf("CoreMark/MHz  : %u.%03u (%u iterations)\n", (unsigned)(score / 1000),
               (unsigned)(score % 1000), (unsigned)iterations);
    }
    p->portable_id = 0;
}
