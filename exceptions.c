#include <stdint.h>

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
}

void MemManage_Handler(void)
{
}

void BusFault_Handler(void)
{
}

void UsageFault_Handler(void)
{
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
    extern uint32_t systick_millis_count;
    systick_millis_count++;
}
