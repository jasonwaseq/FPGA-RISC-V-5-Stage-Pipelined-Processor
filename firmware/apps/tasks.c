/*
 * Compute server: the host puts data in RAM and a request in the parameter
 * block, runs this program, and reads the answer back.
 *
 *   0xFFF0  task      1 sort 32-bit words (in place)      2 CRC-32 of bytes
 *                     3 sum of words                       4 count primes below n
 *                     5 n x n matrix product: C = A * B (A, B, C consecutive)
 *   0xFFF4  address of the data
 *   0xFFF8  length (words for 1, 3; bytes for 2; n for 4, 5)
 *   0xFFFC  result (2, 3, 4)
 *
 * Exit code 0 = done, 1 = unknown task.
 */
#include "lib.h"

struct request {
    uint32_t task, addr, len, result;
};
#define REQ ((volatile struct request *)0xFFF0)

static void sift(uint32_t *a, uint32_t root, uint32_t n)
{
    uint32_t v = a[root];
    for (uint32_t child; (child = 2 * root + 1) < n; root = child) {
        if (child + 1 < n && a[child + 1] > a[child])
            child++;
        if (a[child] <= v)
            break;
        a[root] = a[child];
    }
    a[root] = v;
}

static void heapsort(uint32_t *a, uint32_t n)
{
    for (uint32_t i = n / 2; i-- > 0;)
        sift(a, i, n);
    for (uint32_t end = n; end-- > 1;) {
        uint32_t t = a[0];
        a[0] = a[end];
        a[end] = t;
        sift(a, 0, end);
    }
}

static uint32_t count_primes(uint8_t *sieve, uint32_t n)
{
    uint32_t count = 0;
    memset(sieve, 1, n);
    for (uint32_t i = 2; i < n; i++) {
        if (!sieve[i])
            continue;
        count++;
        for (uint32_t j = i * i; j < n; j += i)
            sieve[j] = 0;
    }
    return count;
}

static void matmul(const int32_t *a, const int32_t *b, int32_t *c, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++)
        for (uint32_t j = 0; j < n; j++) {
            int32_t sum = 0;
            for (uint32_t k = 0; k < n; k++)
                sum += a[i * n + k] * b[k * n + j];
            c[i * n + j] = sum;
        }
}

int main(void)
{
    uint32_t *data = (uint32_t *)REQ->addr;
    uint32_t n = REQ->len;

    switch (REQ->task) {
    case 1:
        heapsort(data, n);
        break;
    case 2:
        REQ->result = crc32(data, n);
        break;
    case 3: {
        uint32_t sum = 0;
        for (uint32_t i = 0; i < n; i++)
            sum += data[i];
        REQ->result = sum;
        break;
    }
    case 4:
        REQ->result = count_primes((uint8_t *)data, n);
        break;
    case 5:
        matmul((int32_t *)data, (int32_t *)data + n * n, (int32_t *)data + 2 * n * n, n);
        break;
    default:
        return 1;
    }
    return 0;
}
