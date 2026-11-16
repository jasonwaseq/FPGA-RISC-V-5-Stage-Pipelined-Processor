#include "lib.h"

/* ---- Console ---------------------------------------------------------------- */
void putchar_(int c)
{
    while (!(REG32(UART_STATUS) & UART_TX_READY))
        ;
    REG32(UART_DATA) = c;
}

int getchar_nb(void)
{
    int32_t v = REG32(UART_DATA);
    return v < 0 ? -1 : (v & 0xFF);
}

int getchar_(void)
{
    int c;
    while ((c = getchar_nb()) < 0)
        ;
    return c;
}

void puts_(const char *s)
{
    while (*s) {
        if (*s == '\n')
            putchar_('\r');
        putchar_(*s++);
    }
}

static int put_num(uint32_t v, unsigned base, int width, int zero, int neg, int upper)
{
    char buf[12];
    int n = 0, count = 0;
    do {
        unsigned d = v % base;
        buf[n++] = d < 10 ? '0' + d : (upper ? 'A' : 'a') + d - 10;
        v /= base;
    } while (v);
    if (neg && zero) {
        putchar_('-');
        count++;
    }
    for (int pad = width - n - neg; pad > 0; pad--, count++)
        putchar_(zero ? '0' : ' ');
    if (neg && !zero) {
        putchar_('-');
        count++;
    }
    while (n) {
        putchar_(buf[--n]);
        count++;
    }
    return count;
}

int vprintf_(const char *fmt, va_list ap)
{
    int count = 0;
    for (; *fmt; fmt++) {
        if (*fmt != '%') {
            if (*fmt == '\n')
                putchar_('\r');
            putchar_(*fmt);
            count++;
            continue;
        }
        int zero = 0, width = 0, left = 0;
        fmt++;
        if (*fmt == '-') {
            left = 1;
            fmt++;
        }
        if (*fmt == '0') {
            zero = 1;
            fmt++;
        }
        while (*fmt >= '0' && *fmt <= '9')
            width = width * 10 + (*fmt++ - '0');
        while (*fmt == 'l' || *fmt == 'z')
            fmt++;
        switch (*fmt) {
        case 'd':
        case 'i': {
            int32_t v = va_arg(ap, int32_t);
            count += put_num(v < 0 ? -(uint32_t)v : (uint32_t)v, 10, width, zero, v < 0, 0);
            break;
        }
        case 'u':
            count += put_num(va_arg(ap, uint32_t), 10, width, zero, 0, 0);
            break;
        case 'x':
        case 'p':
            count += put_num(va_arg(ap, uint32_t), 16, width, zero, 0, 0);
            break;
        case 'X':
            count += put_num(va_arg(ap, uint32_t), 16, width, zero, 0, 1);
            break;
        case 'c':
            putchar_(va_arg(ap, int));
            count++;
            break;
        case 's': {
            const char *s = va_arg(ap, const char *);
            int len = (int)strlen(s);
            for (int pad = width - len; !left && pad > 0; pad--, count++)
                putchar_(' ');
            puts_(s);
            for (int pad = width - len; left && pad > 0; pad--, count++)
                putchar_(' ');
            count += len;
            break;
        }
        case '%':
            putchar_('%');
            count++;
            break;
        default:
            return count;
        }
    }
    return count;
}

int printf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int n = vprintf_(fmt, ap);
    va_end(ap);
    return n;
}

/* Line editor: echo, backspace, returns the length without the newline. */
int readline(char *buf, int max)
{
    int n = 0;
    for (;;) {
        int c = getchar_();
        if (c == '\r' || c == '\n') {
            puts_("\n");
            buf[n] = 0;
            return n;
        }
        if (c == 8 || c == 127) {
            if (n) {
                n--;
                puts_("\b \b");
            }
        } else if (c >= 32 && n < max - 1) {
            buf[n++] = c;
            putchar_(c);
        }
    }
}

/* ---- Strings and memory ------------------------------------------------------- */
void *memcpy(void *dst, const void *src, size_t n)
{
    uint8_t *d = dst;
    const uint8_t *s = src;
    if ((((uintptr_t)d | (uintptr_t)s) & 3) == 0) {
        uint32_t *dw = dst;
        const uint32_t *sw = src;
        for (; n >= 4; n -= 4)
            *dw++ = *sw++;
        d = (uint8_t *)dw;
        s = (const uint8_t *)sw;
    }
    while (n--)
        *d++ = *s++;
    return dst;
}

void *memmove(void *dst, const void *src, size_t n)
{
    uint8_t *d = dst;
    const uint8_t *s = src;
    if (d <= s || d >= s + n)
        return memcpy(dst, src, n);
    while (n--)
        d[n] = s[n];
    return dst;
}

void *memset(void *dst, int c, size_t n)
{
    uint8_t *d = dst;
    while (n--)
        *d++ = c;
    return dst;
}

int memcmp(const void *a, const void *b, size_t n)
{
    const uint8_t *x = a, *y = b;
    for (; n--; x++, y++)
        if (*x != *y)
            return *x - *y;
    return 0;
}

size_t strlen(const char *s)
{
    size_t n = 0;
    while (s[n])
        n++;
    return n;
}

int strcmp(const char *a, const char *b)
{
    while (*a && *a == *b) {
        a++;
        b++;
    }
    return (uint8_t)*a - (uint8_t)*b;
}

uint32_t parse_num(const char *s, int *ok)
{
    uint32_t v = 0;
    int base = 10, digits = 0;
    if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
        base = 16;
        s += 2;
    }
    for (; *s; s++, digits++) {
        int d;
        if (*s >= '0' && *s <= '9')
            d = *s - '0';
        else if (base == 16 && (*s | 32) >= 'a' && (*s | 32) <= 'f')
            d = (*s | 32) - 'a' + 10;
        else {
            digits = 0;
            break;
        }
        v = v * base + d;
    }
    if (ok)
        *ok = digits > 0;
    return v;
}

/* ---- Time ----------------------------------------------------------------------- */
uint64_t rdcycle64(void)
{
    uint32_t hi, lo, hi2;
    do {
        hi = csr_read(mcycleh);
        lo = csr_read(mcycle);
        hi2 = csr_read(mcycleh);
    } while (hi != hi2);
    return ((uint64_t)hi << 32) | lo;
}

uint64_t rdinstret64(void)
{
    uint32_t hi, lo, hi2;
    do {
        hi = csr_read(minstreth);
        lo = csr_read(minstret);
        hi2 = csr_read(minstreth);
    } while (hi != hi2);
    return ((uint64_t)hi << 32) | lo;
}

uint32_t time_ms(void)
{
    return (uint32_t)(rdcycle64() / (clk_hz() / 1000));
}

uint32_t crc32(const void *data, uint32_t n)
{
    const uint8_t *p = data;
    uint32_t crc = 0xFFFFFFFF;
    while (n--) {
        crc ^= *p++;
        for (int i = 0; i < 8; i++)
            crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
    }
    return ~crc;
}

/* ---- Interrupts ------------------------------------------------------------------- */
extern void trap_entry(void);

void irq_init(void)
{
    csr_write(mtvec, (uint32_t)trap_entry);
}

__attribute__((weak)) void irq_handler(uint32_t cause)
{
    (void)cause;
}
