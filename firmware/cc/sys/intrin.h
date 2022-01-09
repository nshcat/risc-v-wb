#pragma once
#include "device.h"

#define __force_inline inline __attribute__((always_inline))

__force_inline
uint32_t __rdtime()
{
    uint32_t result;
    asm __volatile__("rdtime %0" : "=r"(result));
    return result;
}

__force_inline
uint32_t __rdtimeh()
{
    uint32_t result;
    asm __volatile__("rdtimeh %0" : "=r"(result));
    return result;
}

/* Read 64bit time counter value, resistant to rdtime overflows */
__force_inline
uint64_t __rdtime64()
{
    // XXX This has to be optimized. GCC implies a lot of memory reads and writes to the stack.
    uint32_t volatile upper, lower, temp;
    do
    {
        upper = __rdtimeh();
        lower = __rdtime();
        temp = __rdtimeh();
    } while (upper != temp);
    
    return (((uint64_t)upper) << 32) | (uint64_t)lower;
}

/* Read 32bit CSR value */
__force_inline
uint32_t __csrr(int csr)
{
    uint32_t result;
    asm ("csrr %0, %1"
        : "=r"(result)
        : "I"(csr));
    return result;
}

/* Write 32bit CSR value */
__force_inline
void __csrw(int csr, uint32_t value)
{
    asm ("csrw %0, %1"
        : /* No outputs */
        : "I"(csr), "r"(value));
}