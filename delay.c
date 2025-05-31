#include "imxrt.h"

volatile uint32_t systick_millis_count = 0;
volatile uint32_t systick_cycle_count = 0;
volatile uint32_t scale_cpu_cycles_to_microseconds = 0;
uint32_t systick_safe_read;

#define SYSTICK_EXT_FREQ 100000

uint32_t micros()
{
    uint32_t smc, scc;
    do {
        uint32_t result;
        __asm__ volatile ("ldrex %0, [%1]" : "=r" (result) : "r" (&systick_safe_read) );
        smc = systick_millis_count;
        scc = systick_cycle_count;

        __asm__ volatile ("strex %0, %2, %1" : "=&r" (result), "=Q" (systick_safe_read) : "r" (1) );
        if (result == 0) {
            break;
        }

    } while (1);

    uint32_t cyccnt = ARM_DWT_CYCCNT;
    asm volatile("" : : : "memory");
    uint32_t ccdelta = cyccnt - scc;
	uint32_t frac = ((uint64_t)ccdelta * scale_cpu_cycles_to_microseconds) >> 32;
	if (frac > 1000) frac = 1000;
	uint32_t usec = 1000*smc + frac;
	return usec;
}

void delay(uint32_t ms)
{
    uint32_t start;
    if (ms == 0) return;

    start = micros();
    while (1) {
        uint32_t now = micros();
        if (now - start >= ms * 1000) {
            break;
        }
    }
}
