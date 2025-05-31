#include "imxrt.h"

extern unsigned long _stextload;
extern unsigned long _stext;
extern unsigned long _etext;
extern unsigned long _sdataload;
extern unsigned long _sdata;
extern unsigned long _edata;
extern unsigned long _sbss;
extern unsigned long _ebss;
extern unsigned long _flexram_bank_config;
extern unsigned long _estack;
extern unsigned long _extram_start;
extern unsigned long _extram_end;

void ResetHandler(void);
extern void NMI_Handler(void);
extern void HardFault_Handler(void);
extern void MemManage_Handler(void);
extern void BusFault_Handler(void);
extern void UsageFault_Handler(void);
extern void SVC_Handler(void);
extern void DebugMon_Handler(void);
extern void PendSV_Handler(void);
extern void SysTick_Handler(void);

__attribute__ ((used, aligned(1024), section(".vectorsram")))
void (* volatile _VectorsRam[NVIC_NUM_INTERRUPTS+16])(void) = {
    ResetHandler,       /* reset handler */
    NMI_Handler,        /* NMI handler */
    HardFault_Handler,  /* hard fault handler */
    MemManage_Handler,  /* memory management fault handler */
    BusFault_Handler,   /* bus fault handler */
    UsageFault_Handler, /* usage fault handler */
    0, 0, 0, 0,         /* reserved */
    SVC_Handler,        /* SVCall handler */
    DebugMon_Handler,   /* debug monitor handler */
    0,                  /* reserved */
    PendSV_Handler,     /* PendSV handler */
    SysTick_Handler,    /* SysTick handler */
};

#define DMAMEM __attribute__ ((section(".dmabuffers"), used))
#define FASTRUN __attribute__ ((section(".fastrun") ))
#define PROGMEM __attribute__((section(".progmem")))
#define FLASHMEM __attribute__((section(".flashmem")))
#define EXTMEM __attribute__((section(".externalram")))

static void memory_copy(uint32_t *dest, const uint32_t *src, uint32_t *dest_end);
static void memory_clear(uint32_t *dest, uint32_t *dest_end);

static void ResetPFD(void);
static void ConfigureMPU(void);
static void ConfigureSysTick(void);
static void ConfigureExternalRAM(void);
static void ResetHandler2(void);

extern int main();

uint8_t external_psram_size = 0;

__attribute__((section(".startup"), naked))
void ResetHandler()
{
    IOMUXC_GPR_GPR17 = (uint32_t)&_flexram_bank_config;
    IOMUXC_GPR_GPR16 = 0x00200007; /* Enable DTCM, ITCM, FlexRAM Config from bank config */
    IOMUXC_GPR_GPR14 = 0x00AA0000; /* 512 KiB DTCM and 512 KiB ITCM */

    /* setup stack pointer */
    __asm__ volatile("mov sp, %0" : : "r"((uint32_t)&_estack) : "memory");

    ResetHandler2();
}

__attribute__((section(".startup"), noinline, noreturn))
void ResetHandler2()
{
    /* data sync barrier */
    __asm__ volatile("dsb":::"memory");

#if 1
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
#endif

    /* Use bandgap-based bias currents for best performance (Page 1175) */
    PMU_MISC0_SET = 1<<3;

#if 1
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
#endif

    /* copy .text and .data sections */
    memory_copy((uint32_t *)&_stext, (const uint32_t *)&_stextload, (uint32_t *)&_etext);
    memory_copy((uint32_t *)&_sdata, (const uint32_t *)&_sdataload, (uint32_t *)&_edata);

    /* clear .bss section */
    memory_clear((uint32_t *)&_sbss, (uint32_t *)&_ebss);

    SCB_CPACR = 0x00F00000; /* enable FPU via Coprocessor Access Control Register */

    unsigned int i;
    /* for now just set all the vectors to null */
    for (i=0; i < NVIC_NUM_INTERRUPTS; i++) NVIC_SET_PRIORITY(i, 128);
    SCB_VTOR = (uint32_t)_VectorsRam; /* Vectors Table offset */


    /* reset Phase Fractional Dividers */
    ResetPFD();

    /* enable following exceptions */
    SCB_SHCSR |= SCB_SHCSR_MEMFAULTENA | SCB_SHCSR_BUSFAULTENA | SCB_SHCSR_USGFAULTENA;

    /* clock configuration */
    /* PIT & GPT timers to run from 24 MHz clock (independent of CPU speed) */
    CCM_CSCMR1 = (CCM_CSCMR1 & ~CCM_CSCMR1_PERCLK_PODF(0x3F)) | CCM_CSCMR1_PERCLK_CLK_SEL;
    /* UARTs run from 24 MHz clock (works if PLL3 off or bypassed) */
    CCM_CSCDR1 = (CCM_CSCDR1 & ~CCM_CSCDR1_UART_CLK_PODF(0x3F)) | CCM_CSCDR1_UART_CLK_SEL;

    /* enable fast GPIO 6,7,8,9 (reference manual fig-2-1) */
    IOMUXC_GPR_GPR26 = 0xFFFFFFFF;
	IOMUXC_GPR_GPR27 = 0xFFFFFFFF;
	IOMUXC_GPR_GPR28 = 0xFFFFFFFF;
	IOMUXC_GPR_GPR29 = 0xFFFFFFFF;

    ConfigureMPU();

    /* invalidate I-cache */
    asm("dsb");
	asm("isb");
	SCB_CACHE_ICIALLU = 0;

    /* enable I and D-cache */
    asm("dsb");
	asm("isb");
	SCB_CCR |= (SCB_CCR_IC | SCB_CCR_DC);

    ConfigureSysTick();

    /* Undo PIT timer usage by ROM startup */
    CCM_CCGR1 |= CCM_CCGR1_PIT(CCM_CCGR_ON);
    PIT_MCR = 0;
	PIT_TCTRL0 = 0;
	PIT_TCTRL1 = 0;
	PIT_TCTRL2 = 0;
	PIT_TCTRL3 = 0;

    /* initialize RTC */
    if (!(SNVS_LPCR & SNVS_LPCR_SRTC_ENV)) {
		SNVS_LPSRTCLR = 1546300800u << 15;
		SNVS_LPSRTCMR = 1546300800u >> 17;
		SNVS_LPCR |= SNVS_LPCR_SRTC_ENV;
	}
	SNVS_HPCR |= SNVS_HPCR_RTC_EN | SNVS_HPCR_HP_TS;

    ConfigureExternalRAM();
    main();

    while (1) {
        /* main() should never return, so we loop here */
        __asm__ volatile("wfi");
    }
}

FLASHMEM void ResetPFD(void)
{
    /* disable all the PFDs */
    /* PLL2:528MHz */
    CCM_ANALOG_PFD_528_SET = (1 << 31) | (1 << 23) | (1 << 15) | (1 << 7);
    /* PFD0:352, PFD1:594, PFD2:396, PFD3:297 MHz */
    CCM_ANALOG_PFD_528 = 0x2018101B;

    /* PLL3:480MHz */
    CCM_ANALOG_PFD_480_SET = (1 << 31) | (1 << 23) | (1 << 15) | (1 << 7);
    /* PFD0:720, PFD1:664, PFD2:508, PFD3:454 MHz */
    CCM_ANALOG_PFD_480 = 0x13110D0C;
}

#define NOEXEC		    SCB_MPU_RASR_XN
#define READONLY	    SCB_MPU_RASR_AP(7)
#define READWRITE	    SCB_MPU_RASR_AP(3)
#define NOACCESS	    SCB_MPU_RASR_AP(0)
#define MEM_CACHE_WT	SCB_MPU_RASR_TEX(0) | SCB_MPU_RASR_C
#define MEM_CACHE_WB	SCB_MPU_RASR_TEX(0) | SCB_MPU_RASR_C | SCB_MPU_RASR_B
#define MEM_CACHE_WBWA	SCB_MPU_RASR_TEX(1) | SCB_MPU_RASR_C | SCB_MPU_RASR_B
#define MEM_NOCACHE	    SCB_MPU_RASR_TEX(1)
#define DEV_NOCACHE	    SCB_MPU_RASR_TEX(2)
#define SIZE_32B	    (SCB_MPU_RASR_SIZE(4) | SCB_MPU_RASR_ENABLE)
#define SIZE_64B	    (SCB_MPU_RASR_SIZE(5) | SCB_MPU_RASR_ENABLE)
#define SIZE_128B	    (SCB_MPU_RASR_SIZE(6) | SCB_MPU_RASR_ENABLE)
#define SIZE_256B	    (SCB_MPU_RASR_SIZE(7) | SCB_MPU_RASR_ENABLE)
#define SIZE_512B	    (SCB_MPU_RASR_SIZE(8) | SCB_MPU_RASR_ENABLE)
#define SIZE_1K		    (SCB_MPU_RASR_SIZE(9) | SCB_MPU_RASR_ENABLE)
#define SIZE_2K		    (SCB_MPU_RASR_SIZE(10) | SCB_MPU_RASR_ENABLE)
#define SIZE_4K		    (SCB_MPU_RASR_SIZE(11) | SCB_MPU_RASR_ENABLE)
#define SIZE_8K		    (SCB_MPU_RASR_SIZE(12) | SCB_MPU_RASR_ENABLE)
#define SIZE_16K	    (SCB_MPU_RASR_SIZE(13) | SCB_MPU_RASR_ENABLE)
#define SIZE_32K	    (SCB_MPU_RASR_SIZE(14) | SCB_MPU_RASR_ENABLE)
#define SIZE_64K	    (SCB_MPU_RASR_SIZE(15) | SCB_MPU_RASR_ENABLE)
#define SIZE_128K	    (SCB_MPU_RASR_SIZE(16) | SCB_MPU_RASR_ENABLE)
#define SIZE_256K	    (SCB_MPU_RASR_SIZE(17) | SCB_MPU_RASR_ENABLE)
#define SIZE_512K	    (SCB_MPU_RASR_SIZE(18) | SCB_MPU_RASR_ENABLE)
#define SIZE_1M		    (SCB_MPU_RASR_SIZE(19) | SCB_MPU_RASR_ENABLE)
#define SIZE_2M		    (SCB_MPU_RASR_SIZE(20) | SCB_MPU_RASR_ENABLE)
#define SIZE_4M		    (SCB_MPU_RASR_SIZE(21) | SCB_MPU_RASR_ENABLE)
#define SIZE_8M		    (SCB_MPU_RASR_SIZE(22) | SCB_MPU_RASR_ENABLE)
#define SIZE_16M	    (SCB_MPU_RASR_SIZE(23) | SCB_MPU_RASR_ENABLE)
#define SIZE_32M	    (SCB_MPU_RASR_SIZE(24) | SCB_MPU_RASR_ENABLE)
#define SIZE_64M	    (SCB_MPU_RASR_SIZE(25) | SCB_MPU_RASR_ENABLE)
#define SIZE_128M	    (SCB_MPU_RASR_SIZE(26) | SCB_MPU_RASR_ENABLE)
#define SIZE_256M	    (SCB_MPU_RASR_SIZE(27) | SCB_MPU_RASR_ENABLE)
#define SIZE_512M	    (SCB_MPU_RASR_SIZE(28) | SCB_MPU_RASR_ENABLE)
#define SIZE_1G		    (SCB_MPU_RASR_SIZE(29) | SCB_MPU_RASR_ENABLE)
#define SIZE_2G		    (SCB_MPU_RASR_SIZE(30) | SCB_MPU_RASR_ENABLE)
#define SIZE_4G		    (SCB_MPU_RASR_SIZE(31) | SCB_MPU_RASR_ENABLE)
#define REGION(n)	    (SCB_MPU_RBAR_REGION(n) | SCB_MPU_RBAR_VALID)

FLASHMEM void ConfigureMPU(void)
{
    SCB_MPU_CTRL = 0; /* disable MPU */

    uint32_t i = 0;
    SCB_MPU_RBAR = 0x00000000 | REGION(i++);
    SCB_MPU_RASR = SCB_MPU_RASR_TEX(0) | NOACCESS | NOEXEC | SIZE_4G;

    SCB_MPU_RBAR = 0x00000000 | REGION(i++); // ITCM
	SCB_MPU_RASR = MEM_NOCACHE | READONLY | SIZE_512K;

    SCB_MPU_RBAR = 0x00000000 | REGION(i++); // trap NULL pointer deref
	SCB_MPU_RASR =  DEV_NOCACHE | NOACCESS | SIZE_32B;

    SCB_MPU_RBAR = 0x00200000 | REGION(i++); // Boot ROM
	SCB_MPU_RASR = MEM_CACHE_WT | READONLY | SIZE_128K;

    SCB_MPU_RBAR = 0x20000000 | REGION(i++); // DTCM
	SCB_MPU_RASR = MEM_NOCACHE | READWRITE | NOEXEC | SIZE_512K;

    SCB_MPU_RBAR = ((uint32_t)&_ebss) | REGION(i++); // trap stack overflow
	SCB_MPU_RASR = SCB_MPU_RASR_TEX(0) | NOACCESS | NOEXEC | SIZE_32B;

    SCB_MPU_RBAR = 0x20200000 | REGION(i++); // RAM (AXI bus)
	SCB_MPU_RASR = MEM_CACHE_WBWA | READWRITE | NOEXEC | SIZE_1M;

    SCB_MPU_RBAR = 0x40000000 | REGION(i++); // Peripherals
	SCB_MPU_RASR = DEV_NOCACHE | READWRITE | NOEXEC | SIZE_64M;

    SCB_MPU_RBAR = 0x60000000 | REGION(i++); // QSPI Flash
	SCB_MPU_RASR = MEM_CACHE_WBWA | READONLY | SIZE_16M;

    SCB_MPU_RBAR = 0x70000000 | REGION(i++); // FlexSPI2
	SCB_MPU_RASR = MEM_CACHE_WBWA | READWRITE | NOEXEC | SIZE_16M;

    SCB_MPU_RBAR = 0x80000000 | REGION(i++); // SEMC: SDRAM, NAND, SRAM, etc
	SCB_MPU_RASR = MEM_CACHE_WBWA | READWRITE | NOEXEC | SIZE_1G;

    /* allow a few cycles for bus writes before enable MPU */
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");

    SCB_MPU_CTRL = SCB_MPU_CTRL_ENABLE;
}

#define SYSTICK_EXT_FREQ 100000
extern volatile uint32_t systick_cycle_count;

void ConfigureSysTick(void)
{
    SYST_RVR = (SYSTICK_EXT_FREQ / 1000) - 1;
    SYST_CVR = 0;
    SYST_CSR = SYST_CSR_TICKINT | SYST_CSR_ENABLE;
    SCB_SHPR3 = 0x20200000; /* Systick, PendSV = priority 32; */
    ARM_DEMCR |= ARM_DEMCR_TRCENA; /* turn on cycle counter */
    systick_cycle_count = ARM_DWT_CYCCNT; /* compiled 0, corrected w/1st systick */
}

#define LUT0(opcode, pads, operand) (FLEXSPI_LUT_INSTRUCTION((opcode), (pads), (operand)))
#define LUT1(opcode, pads, operand) (FLEXSPI_LUT_INSTRUCTION((opcode), (pads), (operand)) << 16)
#define CMD_SDR         FLEXSPI_LUT_OPCODE_CMD_SDR
#define ADDR_SDR        FLEXSPI_LUT_OPCODE_RADDR_SDR
#define READ_SDR        FLEXSPI_LUT_OPCODE_READ_SDR
#define WRITE_SDR       FLEXSPI_LUT_OPCODE_WRITE_SDR
#define DUMMY_SDR       FLEXSPI_LUT_OPCODE_DUMMY_SDR
#define PINS1           FLEXSPI_LUT_NUM_PADS_1
#define PINS4           FLEXSPI_LUT_NUM_PADS_4

FLASHMEM static void flexspi2_command(uint32_t index, uint32_t addr)
{
	FLEXSPI2_IPCR0 = addr;
	FLEXSPI2_IPCR1 = FLEXSPI_IPCR1_ISEQID(index);
	FLEXSPI2_IPCMD = FLEXSPI_IPCMD_TRG;
	while (!(FLEXSPI2_INTR & FLEXSPI_INTR_IPCMDDONE)); /* wait */
	FLEXSPI2_INTR = FLEXSPI_INTR_IPCMDDONE;
}

FLASHMEM static uint32_t flexspi2_psram_id(uint32_t addr)
{
	FLEXSPI2_IPCR0 = addr;
	FLEXSPI2_IPCR1 = FLEXSPI_IPCR1_ISEQID(3) | FLEXSPI_IPCR1_IDATSZ(4);
	FLEXSPI2_IPCMD = FLEXSPI_IPCMD_TRG;
	while (!(FLEXSPI2_INTR & FLEXSPI_INTR_IPCMDDONE)); /* wait */
	uint32_t id = FLEXSPI2_RFDR0;
	FLEXSPI2_INTR = FLEXSPI_INTR_IPCMDDONE | FLEXSPI_INTR_IPRXWA;
	return id & 0xFFFF;
}

FLASHMEM static void ConfigureExternalRAM(void)
{
    /* initialize pins */
    IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_22 = 0x1B0F9;
    IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_23 = 0x110F9;
    IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_24 = 0x1B0F9;
    IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_25 = 0x100F9;
    IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_26 = 0x170F9;
    IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_27 = 0x170F9;
    IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_28 = 0x170F9;
    IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_29 = 0x170F9;

    /* set all to ALT1 */
    IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_22 = 8 | 0x10;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_23 = 8 | 0x10;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_24 = 8 | 0x10;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_25 = 8 | 0x10;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_26 = 8 | 0x10;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_27 = 8 | 0x10;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_28 = 8 | 0x10;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_29 = 8 | 0x10;

    IOMUXC_FLEXSPI2_IPP_IND_DQS_FA_SELECT_INPUT = 1;
    IOMUXC_FLEXSPI2_IPP_IND_IO_FA_BIT0_SELECT_INPUT = 1;
    IOMUXC_FLEXSPI2_IPP_IND_IO_FA_BIT1_SELECT_INPUT = 1;
    IOMUXC_FLEXSPI2_IPP_IND_IO_FA_BIT2_SELECT_INPUT = 1;
    IOMUXC_FLEXSPI2_IPP_IND_IO_FA_BIT3_SELECT_INPUT = 1;
    IOMUXC_FLEXSPI2_IPP_IND_SCK_FA_SELECT_INPUT = 1;

    /* turn on clock */
    CCM_CBCMR = (CCM_CBCMR & ~(CCM_CBCMR_FLEXSPI2_PODF_MASK | CCM_CBCMR_FLEXSPI2_CLK_SEL_MASK))
		| CCM_CBCMR_FLEXSPI2_PODF(5) | CCM_CBCMR_FLEXSPI2_CLK_SEL(3); /* 88 MHz */
	CCM_CCGR7 |= CCM_CCGR7_FLEXSPI2(CCM_CCGR_ON);

    FLEXSPI2_MCR0 |= FLEXSPI_MCR0_MDIS;
	FLEXSPI2_MCR0 = (FLEXSPI2_MCR0 & ~(FLEXSPI_MCR0_AHBGRANTWAIT_MASK
		 | FLEXSPI_MCR0_IPGRANTWAIT_MASK | FLEXSPI_MCR0_SCKFREERUNEN
		 | FLEXSPI_MCR0_COMBINATIONEN | FLEXSPI_MCR0_DOZEEN
		 | FLEXSPI_MCR0_HSEN | FLEXSPI_MCR0_ATDFEN | FLEXSPI_MCR0_ARDFEN
		 | FLEXSPI_MCR0_RXCLKSRC_MASK | FLEXSPI_MCR0_SWRESET))
		| FLEXSPI_MCR0_AHBGRANTWAIT(0xFF) | FLEXSPI_MCR0_IPGRANTWAIT(0xFF)
		| FLEXSPI_MCR0_RXCLKSRC(1) | FLEXSPI_MCR0_MDIS;
	FLEXSPI2_MCR1 = FLEXSPI_MCR1_SEQWAIT(0xFFFF) | FLEXSPI_MCR1_AHBBUSWAIT(0xFFFF);
	FLEXSPI2_MCR2 = (FLEXSPI_MCR2 & ~(FLEXSPI_MCR2_RESUMEWAIT_MASK
		 | FLEXSPI_MCR2_SCKBDIFFOPT | FLEXSPI_MCR2_SAMEDEVICEEN
		 | FLEXSPI_MCR2_CLRLEARNPHASE | FLEXSPI_MCR2_CLRAHBBUFOPT))
		| FLEXSPI_MCR2_RESUMEWAIT(0x20) /*| FLEXSPI_MCR2_SAMEDEVICEEN*/;

    FLEXSPI2_AHBCR = FLEXSPI2_AHBCR & ~(FLEXSPI_AHBCR_READADDROPT | FLEXSPI_AHBCR_PREFETCHEN
		| FLEXSPI_AHBCR_BUFFERABLEEN | FLEXSPI_AHBCR_CACHABLEEN);
	uint32_t mask = (FLEXSPI_AHBRXBUFCR0_PREFETCHEN | FLEXSPI_AHBRXBUFCR0_PRIORITY_MASK
		| FLEXSPI_AHBRXBUFCR0_MSTRID_MASK | FLEXSPI_AHBRXBUFCR0_BUFSZ_MASK);
	FLEXSPI2_AHBRXBUF0CR0 = (FLEXSPI2_AHBRXBUF0CR0 & ~mask)
		| FLEXSPI_AHBRXBUFCR0_PREFETCHEN | FLEXSPI_AHBRXBUFCR0_BUFSZ(64);
	FLEXSPI2_AHBRXBUF1CR0 = (FLEXSPI2_AHBRXBUF0CR0 & ~mask)
		| FLEXSPI_AHBRXBUFCR0_PREFETCHEN | FLEXSPI_AHBRXBUFCR0_BUFSZ(64);
	FLEXSPI2_AHBRXBUF2CR0 = mask;
	FLEXSPI2_AHBRXBUF3CR0 = mask;

    FLEXSPI2_IPRXFCR = (FLEXSPI_IPRXFCR & 0xFFFFFFC0) | FLEXSPI_IPRXFCR_CLRIPRXF;
    FLEXSPI2_IPTXFCR = (FLEXSPI_IPTXFCR & 0xFFFFFFC0) | FLEXSPI_IPTXFCR_CLRIPTXF;

    FLEXSPI2_INTEN = 0;
    FLEXSPI2_FLSHA1CR0 = 0x2000;
    FLEXSPI2_FLSHA1CR1 = FLEXSPI_FLSHCR1_CSINTERVAL(2)
		| FLEXSPI_FLSHCR1_TCSH(3) | FLEXSPI_FLSHCR1_TCSS(3);
	FLEXSPI2_FLSHA1CR2 = FLEXSPI_FLSHCR2_AWRSEQID(6) | FLEXSPI_FLSHCR2_AWRSEQNUM(0)
		| FLEXSPI_FLSHCR2_ARDSEQID(5) | FLEXSPI_FLSHCR2_ARDSEQNUM(0);
    FLEXSPI2_FLSHA2CR0 = 0x2000;

    FLEXSPI2_FLSHA2CR1 = FLEXSPI_FLSHCR1_CSINTERVAL(2)
		| FLEXSPI_FLSHCR1_TCSH(3) | FLEXSPI_FLSHCR1_TCSS(3);
	FLEXSPI2_FLSHA2CR2 = FLEXSPI_FLSHCR2_AWRSEQID(6) | FLEXSPI_FLSHCR2_AWRSEQNUM(0)
		| FLEXSPI_FLSHCR2_ARDSEQID(5) | FLEXSPI_FLSHCR2_ARDSEQNUM(0);

	FLEXSPI2_MCR0 &= ~FLEXSPI_MCR0_MDIS;

    FLEXSPI2_LUTKEY = FLEXSPI_LUTKEY_VALUE;
	FLEXSPI2_LUTCR = FLEXSPI_LUTCR_UNLOCK;
	volatile uint32_t *luttable = &FLEXSPI2_LUT0;
	for (int i=0; i < 64; i++) luttable[i] = 0;
	FLEXSPI2_MCR0 |= FLEXSPI_MCR0_SWRESET;
	while (FLEXSPI2_MCR0 & FLEXSPI_MCR0_SWRESET) ;

	FLEXSPI2_LUTKEY = FLEXSPI_LUTKEY_VALUE;
	FLEXSPI2_LUTCR = FLEXSPI_LUTCR_UNLOCK;

    /* configure LUTs */
    /* cmd index 0 = exit QPI mode */
    FLEXSPI2_LUT0 = LUT0(CMD_SDR, PINS4, 0xF5);
    /* cmd index 1 = reset enable */
    FLEXSPI2_LUT4 = LUT0(CMD_SDR, PINS1, 0x66);
    /* cmd index 2 = reset */
	FLEXSPI2_LUT8 = LUT0(CMD_SDR, PINS1, 0x99);
	/* cmd index 3 = read ID bytes */
	FLEXSPI2_LUT12 = LUT0(CMD_SDR, PINS1, 0x9F) | LUT1(DUMMY_SDR, PINS1, 24);
	FLEXSPI2_LUT13 = LUT0(READ_SDR, PINS1, 1);
	/* cmd index 4 = enter QPI mode */
	FLEXSPI2_LUT16 = LUT0(CMD_SDR, PINS1, 0x35);
	/* cmd index 5 = read QPI */
	FLEXSPI2_LUT20 = LUT0(CMD_SDR, PINS4, 0xEB) | LUT1(ADDR_SDR, PINS4, 24);
	FLEXSPI2_LUT21 = LUT0(DUMMY_SDR, PINS4, 6) | LUT1(READ_SDR, PINS4, 1);
	/* cmd index 6 = write QPI */
	FLEXSPI2_LUT24 = LUT0(CMD_SDR, PINS4, 0x38) | LUT1(ADDR_SDR, PINS4, 24);
	FLEXSPI2_LUT25 = LUT0(WRITE_SDR, PINS4, 1);

    /* look for the first PSRAM chip */
    flexspi2_command(0, 0);
    flexspi2_command(1, 0);
    flexspi2_command(2, 0);
    if (flexspi2_psram_id(0) == 0x5D0D) {
        /* first PSRAM chip is present, look for a second PSRAM chip */
        flexspi2_command(4, 0);
        flexspi2_command(0, 0x800000);
        flexspi2_command(1, 0x800000);
        flexspi2_command(2, 0x800000);
        if (flexspi2_psram_id(0x800000) == 0x5D0D) {
            /* Two PSRAM chips are present, 16 MByte */
            external_psram_size = 16;
        } else {
            /* One PSRAM chip is present, 8 MByte */
            external_psram_size = 8;
        }
    } else {
        /* No PSRAM chip is present */
        external_psram_size = 0;
    }

    main();
}

__attribute__((section(".startup"), noinline))
static void memory_copy(uint32_t *dest, const uint32_t *src, uint32_t *dest_end)
{
#if 0
	if (dest == src) return;
	do {
		*dest++ = *src++;
	} while (dest < dest_end);
#else
	asm volatile(
	"	cmp	%[src], %[dest]		\n"
	"	beq.n	2f			\n"
	"1:	ldr.w	r3, [%[src]], #4	\n"
	"	str.w	r3, [%[dest]], #4	\n"
	"	cmp	%[end], %[dest]		\n"
	"	bhi.n	1b			\n"
	"2:					\n"
	: [dest] "+r" (dest), [src] "+r" (src) : [end] "r" (dest_end) : "r3", "memory");
#endif
}

__attribute__((section(".startup"), noinline))
static void memory_clear(uint32_t *dest, uint32_t *dest_end)
{
#if 0
	while (dest < dest_end) {
		*dest++ = 0;
	}
#else
	asm volatile(
	"	ldr	r3, =0			\n"
	"1:	str.w	r3, [%[dest]], #4	\n"
	"	cmp	%[end], %[dest]		\n"
	"	bhi.n	1b			\n"
	: [dest] "+r" (dest) : [end] "r" (dest_end) : "r3", "memory");
#endif
}
