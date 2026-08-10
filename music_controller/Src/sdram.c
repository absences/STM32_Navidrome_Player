#include "sdram.h"
#include "stm32h743xx.h"

static void gpio_af12(GPIO_TypeDef *port, uint32_t pins)
{
    for (uint32_t pin = 0U; pin < 16U; ++pin) {
        if ((pins & (1UL << pin)) == 0U) continue;
        port->MODER = (port->MODER & ~(3UL << (pin * 2U))) |
                      (2UL << (pin * 2U));
        port->OTYPER &= ~(1UL << pin);
        port->OSPEEDR |= 3UL << (pin * 2U);
        port->PUPDR = (port->PUPDR & ~(3UL << (pin * 2U))) |
                      (1UL << (pin * 2U));
        uint32_t index = pin >> 3U;
        uint32_t shift = (pin & 7U) * 4U;
        port->AFR[index] = (port->AFR[index] & ~(0xFUL << shift)) |
                           (12UL << shift);
    }
}

static uint32_t wait_ready(void)
{
    uint32_t start = DWT->CYCCNT;
    while ((FMC_Bank5_6_R->SDSR & (1UL << 5U)) != 0U) {
        if ((uint32_t)(DWT->CYCCNT - start) > SystemCoreClock / 10U)
            return 0U;
    }
    return 1U;
}

static uint32_t command(uint32_t mode, uint32_t refresh, uint32_t mode_reg)
{
    if (!wait_ready()) return 0U;
    FMC_Bank5_6_R->SDCMR = mode | FMC_SDCMR_CTB1 |
                         ((refresh - 1U) << FMC_SDCMR_NRFS_Pos) |
                         (mode_reg << FMC_SDCMR_MRD_Pos);
    return wait_ready();
}

static uint32_t sdram_init(void)
{
    /* Match the board reference memory attributes.  Without an explicit
     * normal-memory MPU region, Cortex-M7 accesses to 0xC0000000 can fault
     * even after the FMC command sequence has completed. */
    __DMB();
    MPU->CTRL = 0U;
    MPU->RNR = 2U;
    MPU->RBAR = SDRAM_BASE_ADDR;
    MPU->RASR = (3UL << MPU_RASR_AP_Pos) |
                MPU_RASR_C_Msk | MPU_RASR_B_Msk |
                (24UL << MPU_RASR_SIZE_Pos) | MPU_RASR_ENABLE_Msk;
    MPU->CTRL = MPU_CTRL_PRIVDEFENA_Msk | MPU_CTRL_ENABLE_Msk;
    __DSB();
    __ISB();

    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOCEN | RCC_AHB4ENR_GPIODEN |
                    RCC_AHB4ENR_GPIOEEN | RCC_AHB4ENR_GPIOFEN |
                    RCC_AHB4ENR_GPIOGEN;
    RCC->APB4ENR |= RCC_APB4ENR_SYSCFGEN;
    RCC->AHB3ENR |= RCC_AHB3ENR_FMCEN;
    (void)RCC->AHB3ENR;
    RCC->AHB3RSTR |= RCC_AHB3RSTR_FMCRST;
    RCC->AHB3RSTR &= ~RCC_AHB3RSTR_FMCRST;
    SYSCFG->PMCR &= ~(SYSCFG_PMCR_PC2SO | SYSCFG_PMCR_PC3SO);
    FMC_Bank1_R->BTCR[0] |= FMC_BCR1_FMCEN;

    gpio_af12(GPIOC, (1UL << 0) | (1UL << 2) | (1UL << 3));
    gpio_af12(GPIOD, (1UL << 0) | (1UL << 1) | (1UL << 8) | (1UL << 9) |
                      (1UL << 10) | (1UL << 14) | (1UL << 15));
    gpio_af12(GPIOE, (1UL << 0) | (1UL << 1) | (1UL << 7) | (1UL << 8) |
                      (1UL << 9) | (1UL << 10) | (1UL << 11) | (1UL << 12) |
                      (1UL << 13) | (1UL << 14) | (1UL << 15));
    gpio_af12(GPIOF, (1UL << 0) | (1UL << 1) | (1UL << 2) | (1UL << 3) |
                      (1UL << 4) | (1UL << 5) | (1UL << 11) | (1UL << 12) |
                      (1UL << 13) | (1UL << 14) | (1UL << 15));
    gpio_af12(GPIOG, (1UL << 0) | (1UL << 1) | (1UL << 2) | (1UL << 4) |
                      (1UL << 5) | (1UL << 8) | (1UL << 15));

    /* W9825G6K: 8192 rows, 512 columns, four banks, x16, CAS 2.
     * The project runs HCLK3 at 100 MHz; SDCLK=/2 gives a conservative 50 MHz. */
    FMC_Bank5_6_R->SDCR[0] = FMC_SDCRx_NC_0 | FMC_SDCRx_NR_1 |
                              FMC_SDCRx_MWID_0 |
                              FMC_SDCRx_NB | FMC_SDCRx_CAS_1 |
                              FMC_SDCRx_SDCLK_1 | FMC_SDCRx_RBURST;
    FMC_Bank5_6_R->SDTR[0] = ((2U - 1U) << FMC_SDTRx_TMRD_Pos) |
                              ((8U - 1U) << FMC_SDTRx_TXSR_Pos) |
                              ((6U - 1U) << FMC_SDTRx_TRAS_Pos) |
                              ((6U - 1U) << FMC_SDTRx_TRC_Pos) |
                              ((2U - 1U) << FMC_SDTRx_TWR_Pos) |
                              ((2U - 1U) << FMC_SDTRx_TRP_Pos) |
                              ((2U - 1U) << FMC_SDTRx_TRCD_Pos);

    if (!command(1U, 1U, 0U)) return 0U; /* clock enable */
    uint32_t start = DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT - start) < SystemCoreClock / 2000U) { }
    if (!command(2U, 1U, 0U)) return 0U; /* precharge all */
    if (!command(3U, 8U, 0U)) return 0U; /* auto refresh */
    if (!command(4U, 1U, 0x220U)) return 0U; /* CAS 2, single write */
    FMC_Bank5_6_R->SDRTR = 370U << FMC_SDRTR_COUNT_Pos;
    return 1U;
}

uint32_t SDRAM_InitAndTest(void)
{
    if (!sdram_init()) return 0U;
    volatile uint32_t *memory = (volatile uint32_t *)SDRAM_BASE_ADDR;
    static const uint32_t offsets[] = {
        0U, 1U, 1024U, 0x10000U, 0x100000U, 0x3FFFFFU, 0x7FFFFFU
    };
    for (uint32_t i = 0U; i < sizeof(offsets) / sizeof(offsets[0]); ++i) {
        memory[offsets[i]] = 0xA5A50000UL ^ offsets[i];
        __DSB();
    }
    __DSB();
    for (uint32_t i = 0U; i < sizeof(offsets) / sizeof(offsets[0]); ++i)
        if (memory[offsets[i]] != (0xA5A50000UL ^ offsets[i])) return 0U;
    for (uint32_t i = 0U; i < sizeof(offsets) / sizeof(offsets[0]); ++i)
        memory[offsets[i]] = 0x5A5AFFFFUL ^ offsets[i];
    __DSB();
    for (uint32_t i = 0U; i < sizeof(offsets) / sizeof(offsets[0]); ++i)
        if (memory[offsets[i]] != (0x5A5AFFFFUL ^ offsets[i])) return 0U;
    return 1U;
}
