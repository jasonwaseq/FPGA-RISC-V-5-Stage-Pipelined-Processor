/* Minimal freestanding runtime for programs on the SoC. */
#ifndef LIB_H
#define LIB_H

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include "soc.h"

/* Console (UART, polled) */
void putchar_(int c);
int  getchar_(void);            /* blocks */
int  getchar_nb(void);          /* -1 if nothing is waiting */
void puts_(const char *s);      /* no newline appended */
int  printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
int  vprintf_(const char *fmt, va_list ap);
int  readline(char *buf, int max);

/* Strings and memory */
void  *memcpy(void *dst, const void *src, size_t n);
void  *memmove(void *dst, const void *src, size_t n);
void  *memset(void *dst, int c, size_t n);
int    memcmp(const void *a, const void *b, size_t n);
size_t strlen(const char *s);
int    strcmp(const char *a, const char *b);
uint32_t parse_num(const char *s, int *ok);     /* decimal or 0x hex */

/* Time: the cycle counter is the time base */
static inline uint32_t clk_hz(void) { return REG32(SYS_CLK_HZ); }
uint64_t rdcycle64(void);
uint64_t rdinstret64(void);
uint32_t time_ms(void);

uint32_t crc32(const void *data, uint32_t n);

/* Control and status registers */
#define csr_read(csr) ({ uint32_t v_; __asm__ volatile("csrr %0, " #csr : "=r"(v_)); v_; })
#define csr_write(csr, v) __asm__ volatile("csrw " #csr ", %0" : : "r"((uint32_t)(v)))
#define csr_set(csr, v)   __asm__ volatile("csrs " #csr ", %0" : : "r"((uint32_t)(v)))
#define csr_clear(csr, v) __asm__ volatile("csrc " #csr ", %0" : : "r"((uint32_t)(v)))

#define MSTATUS_MIE     0x008
#define MIE_MSIE        0x008
#define MIE_MTIE        0x080
#define MIE_MEIE        0x800
#define CAUSE_IRQ       0x80000000u
#define IRQ_SOFT        3
#define IRQ_TIMER       7
#define IRQ_EXT         11

/* Interrupts. irq_init() installs a trap entry that calls irq_handler() for
 * interrupts and hands every exception to the boot ROM's crash reporter. */
void irq_init(void);
void irq_handler(uint32_t cause);       /* provided by the program */

void exit(int code) __attribute__((noreturn));

#endif
