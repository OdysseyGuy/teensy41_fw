#include "imxrt.h"

extern void delay(int ms);

int main() {
    /* set B0_03 to ALT5 (GPIO) */
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03 = 5;
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_03 = IOMUXC_PAD_DSE(7);
    IOMUXC_GPR_GPR27 = 0xFFFFFFFF;
    GPIO7_GDIR |= (1 << 3); /* set GPOI2_IO03 as output */
    
    while (1) {
        GPIO7_DR_SET = (1 << 3); /* set GPIO2_IO03 high */
        delay(500);
        GPIO7_DR_CLEAR = (1 << 3); /* set GPIO2_IO03 low */
        delay(500);
    }
}
