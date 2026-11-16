/* CoreMark port for the RV32IM SoC (bare metal, no floating point, no malloc). */
#ifndef CORE_PORTME_H
#define CORE_PORTME_H

#include <stddef.h>
#include <stdint.h>

#define HAS_FLOAT         0
#define HAS_TIME_H        0
#define USE_CLOCK         0
#define HAS_STDIO         0
#define HAS_PRINTF        0

#ifndef COMPILER_VERSION
#define COMPILER_VERSION  "GCC" __VERSION__
#endif
#ifndef COMPILER_FLAGS
#define COMPILER_FLAGS    FLAGS_STR
#endif
#ifndef MEM_LOCATION
#define MEM_LOCATION      "STACK"
#endif

typedef int16_t  ee_s16;
typedef uint16_t ee_u16;
typedef int32_t  ee_s32;
typedef uint8_t  ee_u8;
typedef uint32_t ee_u32;
typedef uint32_t ee_ptr_int;
typedef size_t   ee_size_t;
typedef uint64_t CORE_TICKS;        /* clock cycles */

#define align_mem(x)      (void *)(4 + (((ee_ptr_int)(x)-1) & ~3))

#define SEED_METHOD       SEED_VOLATILE
#define MEM_METHOD        MEM_STACK
#define MULTITHREAD       1
#define USE_PTHREAD       0
#define USE_FORK          0
#define USE_SOCKET        0
#define MAIN_HAS_NOARGC   1
#define MAIN_HAS_NORETURN 0

extern ee_u32 default_num_contexts;

typedef struct CORE_PORTABLE_S {
    ee_u8 portable_id;
} core_portable;

void portable_init(core_portable *p, int *argc, char *argv[]);
void portable_fini(core_portable *p);

int ee_printf(const char *fmt, ...);

#if !defined(PROFILE_RUN) && !defined(PERFORMANCE_RUN) && !defined(VALIDATION_RUN)
#if (TOTAL_DATA_SIZE == 1200)
#define PROFILE_RUN 1
#elif (TOTAL_DATA_SIZE == 2000)
#define PERFORMANCE_RUN 1
#else
#define VALIDATION_RUN 1
#endif
#endif

#endif
