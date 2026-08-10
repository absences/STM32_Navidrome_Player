#include "wm8978_playback.h"
#include "stm32h743xx.h"
#include "sdram.h"

#define WM8978_ADDR       0x1AU
#define CTRL_SCL_PIN      12U /* PD12 / I2C4_SCL, reference board wiring */
#define CTRL_SDA_PIN      13U /* PD13 / I2C4_SDA, reference board wiring */

#define REG_RESET         0x00U
#define REG_POWER1        0x01U
#define REG_POWER2        0x02U
#define REG_POWER3        0x03U
#define REG_AUDIO_IF      0x04U
#define REG_CLOCK         0x06U
#define REG_DAC           0x0AU
#define REG_DACL          0x0BU
#define REG_DACR          0x0CU
#define REG_BEEP          0x2BU
#define REG_MIXER_L       0x32U
#define REG_MIXER_R       0x33U
#define REG_LOUT1         0x34U
#define REG_ROUT1         0x35U
#define REG_LOUT2         0x36U
#define REG_ROUT2         0x37U

static volatile uint32_t g_playing = 1U;
static volatile uint8_t g_volume = 15U;
static volatile uint32_t g_codec_ready = 0U;
#define PCM_RING_FRAMES (1U * 1024U * 1024U)
/* SDRAM layout: 0..14 MiB cache A, 14..28 MiB cache B, 28..32 MiB PCM. */
static int16_t *const g_pcm =
    (int16_t *)(SDRAM_BASE_ADDR + 28UL * 1024UL * 1024UL);
static volatile uint32_t g_pcm_read = 0U;
static volatile uint32_t g_pcm_write = 0U;
static volatile uint32_t g_tx_slot = 0U;
static volatile uint32_t g_played_seconds = 0U;
static volatile uint32_t g_played_frame_remainder = 0U;
static volatile uint32_t g_audio_started_once = 0U;
static int16_t g_tx_left;
static int16_t g_tx_right;
static uint32_t g_sample_rate = 44100U;

static void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = (SystemCoreClock / 1000000U) * us;
    while ((uint32_t)(DWT->CYCCNT - start) < ticks) { }
}

static void scl(uint32_t high)
{
    GPIOD->BSRR = high ? (1UL << CTRL_SCL_PIN)
                       : (1UL << (CTRL_SCL_PIN + 16U));
}

static void sda(uint32_t high)
{
    GPIOD->BSRR = high ? (1UL << CTRL_SDA_PIN)
                       : (1UL << (CTRL_SDA_PIN + 16U));
}

static uint32_t sda_high(void)
{
    return (GPIOD->IDR & (1UL << CTRL_SDA_PIN)) != 0U;
}

static void control_bus_init(void)
{
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIODEN;
    (void)RCC->AHB4ENR;
    GPIOD->BSRR = (1UL << CTRL_SCL_PIN) | (1UL << CTRL_SDA_PIN);
    GPIOD->MODER &= ~((3UL << (CTRL_SCL_PIN * 2U)) |
                      (3UL << (CTRL_SDA_PIN * 2U)));
    GPIOD->MODER |= (1UL << (CTRL_SCL_PIN * 2U)) |
                    (1UL << (CTRL_SDA_PIN * 2U));
    GPIOD->OTYPER |= (1UL << CTRL_SCL_PIN) | (1UL << CTRL_SDA_PIN);
    GPIOD->PUPDR &= ~((3UL << (CTRL_SCL_PIN * 2U)) |
                      (3UL << (CTRL_SDA_PIN * 2U)));
    GPIOD->PUPDR |= (1UL << (CTRL_SCL_PIN * 2U)) |
                    (1UL << (CTRL_SDA_PIN * 2U));
    scl(1U);
    sda(1U);
}

static uint32_t control_write(const uint8_t *bytes, uint32_t count)
{
    uint32_t acknowledged = 1U;

    sda(1U); scl(1U); delay_us(5U);
    sda(0U); delay_us(5U); scl(0U);

    for (uint32_t index = 0U; index <= count; ++index) {
        uint8_t value = index == 0U ? (uint8_t)(WM8978_ADDR << 1U)
                                    : bytes[index - 1U];
        for (uint32_t bit = 0U; bit < 8U; ++bit) {
            sda((value & 0x80U) != 0U);
            delay_us(2U); scl(1U); delay_us(5U); scl(0U); delay_us(2U);
            value <<= 1U;
        }
        sda(1U); delay_us(2U); scl(1U); delay_us(3U);
        if (sda_high()) acknowledged = 0U;
        delay_us(2U); scl(0U);
    }

    sda(0U); delay_us(2U); scl(1U); delay_us(5U);
    sda(1U); delay_us(5U);
    return acknowledged;
}

static uint32_t write_reg(uint8_t reg, uint16_t value)
{
    uint8_t bytes[2];
    value &= 0x01FFU;
    bytes[0] = (uint8_t)((reg << 1U) | ((value >> 8U) & 1U));
    bytes[1] = (uint8_t)value;
    for (uint32_t attempt = 0U; attempt < 3U; ++attempt) {
        if (control_write(bytes, 2U)) return 1U;
        delay_us(2000U);
    }
    return 0U;
}

static void sai_gpio_init(void)
{
    const uint32_t pins = (1UL << 2U) | (1UL << 4U) |
                          (1UL << 5U) | (1UL << 6U);
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOEEN;
    (void)RCC->AHB4ENR;
    GPIOE->MODER &= ~((3UL << (2U * 2U)) | (3UL << (4U * 2U)) |
                      (3UL << (5U * 2U)) | (3UL << (6U * 2U)));
    GPIOE->MODER |= (2UL << (2U * 2U)) | (2UL << (4U * 2U)) |
                    (2UL << (5U * 2U)) | (2UL << (6U * 2U));
    GPIOE->OSPEEDR |= (3UL << (2U * 2U)) | (3UL << (4U * 2U)) |
                      (3UL << (5U * 2U)) | (3UL << (6U * 2U));
    GPIOE->PUPDR &= ~((3UL << (2U * 2U)) | (3UL << (4U * 2U)) |
                      (3UL << (5U * 2U)) | (3UL << (6U * 2U)));
    GPIOE->AFR[0] &= ~((0xFUL << (2U * 4U)) | (0xFUL << (4U * 4U)) |
                       (0xFUL << (5U * 4U)) | (0xFUL << (6U * 4U)));
    GPIOE->AFR[0] |= (6UL << (2U * 4U)) | (6UL << (4U * 4U)) |
                     (6UL << (5U * 4U)) | (6UL << (6U * 4U));
    (void)pins;
}

static void sai_init(uint32_t sample_rate)
{
    /* PLL2P is sample_rate * 1024.  With SAI MCKDIV=4 this produces
     * MCLK=256fs and an exact LRCLK for both 44.1 and 48 kHz families. */
    uint64_t multiplier_x8192 =
        ((uint64_t)sample_rate * 2048ULL * 8192ULL + 500000ULL) /
        1000000ULL;
    uint32_t pll_n = (uint32_t)(multiplier_x8192 / 8192ULL);
    uint32_t pll_frac = (uint32_t)(multiplier_x8192 % 8192ULL);

    RCC->APB2ENR |= RCC_APB2ENR_SAI1EN;
    (void)RCC->APB2ENR;
    SAI1_Block_A->IMR = 0U;
    SAI1_Block_A->CR1 &= ~SAI_xCR1_SAIEN;
    RCC->CR &= ~RCC_CR_PLL2ON;
    while ((RCC->CR & RCC_CR_PLL2RDY) != 0U) { }
    RCC->PLLCKSELR = (RCC->PLLCKSELR &
                     ~(RCC_PLLCKSELR_PLLSRC_Msk | RCC_PLLCKSELR_DIVM2_Msk)) |
                    RCC_PLLCKSELR_PLLSRC_HSI |
                    (16UL << RCC_PLLCKSELR_DIVM2_Pos);
    RCC->PLL2DIVR = ((pll_n - 1UL) << RCC_PLL2DIVR_N2_Pos) |
                    (7UL << RCC_PLL2DIVR_P2_Pos) |
                    (1UL << RCC_PLL2DIVR_Q2_Pos) |
                    (1UL << RCC_PLL2DIVR_R2_Pos);
    RCC->PLL2FRACR = pll_frac << RCC_PLL2FRACR_FRACN2_Pos;
    RCC->PLLCFGR = (RCC->PLLCFGR &
                   ~(RCC_PLLCFGR_PLL2RGE_Msk | RCC_PLLCFGR_PLL2VCOSEL)) |
                  RCC_PLLCFGR_PLL2RGE_1 | RCC_PLLCFGR_PLL2FRACEN |
                  RCC_PLLCFGR_DIVP2EN;
    RCC->CR |= RCC_CR_PLL2ON;
    while ((RCC->CR & RCC_CR_PLL2RDY) == 0U) { }

    RCC->D2CCIP1R = (RCC->D2CCIP1R & ~RCC_D2CCIP1R_SAI1SEL_Msk) |
                    RCC_D2CCIP1R_SAI1SEL_0;
    SAI1_Block_A->CR1 = 0U;
    /* Request service while the FIFO is half empty.  Servicing only an empty
     * FIFO one word at a time is very sensitive to ESP/UART processing and
     * produces audible under-run clicks. */
    SAI1_Block_A->CR2 = SAI_xCR2_FTH_1;
    SAI1_Block_A->FRCR = 31U | (15U << SAI_xFRCR_FSALL_Pos) |
                         SAI_xFRCR_FSDEF | SAI_xFRCR_FSOFF;
    SAI1_Block_A->SLOTR = SAI_xSLOTR_SLOTSZ_0 |
                          SAI_xSLOTR_NBSLOT_0 |
                          (3UL << SAI_xSLOTR_SLOTEN_Pos);
    SAI1_Block_A->CLRFR = 0xFFFFFFFFU;
    SAI1_Block_A->CR1 = SAI_xCR1_DS_2 |
                        SAI_xCR1_CKSTR |
                        SAI_xCR1_MCKEN |
                        (4UL << SAI_xCR1_MCKDIV_Pos);
    /* Start FIFO interrupts only after the first PCM frames are queued. */
    SAI1_Block_A->IMR = 0U;
    NVIC_SetPriority(SAI1_IRQn, 5U);
    NVIC_EnableIRQ(SAI1_IRQn);
    SAI1_Block_A->CR1 |= SAI_xCR1_SAIEN;
}

uint32_t WM8978_PlaybackInit(void)
{
    control_bus_init();
    delay_us(50000U);

    if (!write_reg(REG_RESET, 0U)) return 0U;
    delay_us(10000U);
    if (!write_reg(REG_AUDIO_IF, 0x0010U) ||
        !write_reg(REG_CLOCK, 0x0000U) ||
        !write_reg(REG_POWER1, 0x000BU) ||
        !write_reg(REG_POWER2, 0x0180U) ||
        !write_reg(REG_POWER3, 0x006FU) ||
        !write_reg(REG_DAC, 0x0000U) ||
        /* BTL speaker output: invert ROUT2 and enable speaker boost plus
         * thermal protection, while routing both DACs to the mixers. */
        !write_reg(REG_BEEP, 0x0010U) ||
        /* Keep thermal protection enabled, but do not use the +1.5 dB
         * speaker boost: full-scale decoded MP3 can otherwise clip OUT2. */
        /* BTL uses identical mono samples on LDAC/RDAC and inverts ROUT2.
         * Do not cross-sum the DACs here: L+R in both analogue mixers can
         * exceed full scale and makes normal music sound like heavy noise. */
        !write_reg(49U, 0x0002U) ||
        !write_reg(REG_MIXER_L, 0x0001U) ||
        !write_reg(REG_MIXER_R, 0x0001U) ||
        !write_reg(REG_LOUT1, 32U) ||
        !write_reg(REG_ROUT1, 0x0100U | 32U) ||
        !write_reg(REG_LOUT2, 0x0039U) ||
        !write_reg(REG_ROUT2, 0x0100U | 0x0039U)) return 0U;

    WM8978_SetVolume(g_volume);
    /* Only start the continuously requesting SAI peripheral after the codec
     * has acknowledged every control write.  This keeps a missing codec from
     * creating an interrupt storm that prevents the OLED error from showing. */
    sai_gpio_init();
    sai_init(g_sample_rate);
    g_codec_ready = 1U;
    return 1U;
}

void WM8978_SetSampleRate(uint32_t sample_rate)
{
    if (sample_rate < 8000U || sample_rate > 96000U ||
        sample_rate == g_sample_rate) return;
    __disable_irq();
    g_sample_rate = sample_rate;
    g_tx_slot = 0U;
    sai_init(sample_rate);
    __enable_irq();
}

void WM8978_SetVolume(uint8_t volume)
{
    if (volume < 1U) volume = 1U;
    if (volume > 30U) volume = 30U;
    g_volume = volume;
    /* Once playback is running, do not rewrite the codec output registers.
     * Some WM8978 modules briefly latch/mute OUT2 when these registers are
     * updated while clocks are active.  Runtime volume is applied digitally
     * in the SAI interrupt instead. */
    if (g_codec_ready) return;
    /* Keep the DAC digital path at 0 dB.  Registers 11/12 use an 8-bit
     * attenuation scale where 0x00 is mute and 0xFF is 0 dB; they are not
     * the user-facing 0..63 output volume controls. */
    (void)write_reg(REG_DACL, 0x00FFU);
    (void)write_reg(REG_DACR, 0x01FFU);
    /* OUT1 is the headphone pair; OUT2 is the speaker/secondary pair. */
    /* Fixed analogue stage: 58/63 provides more speaker headroom while
     * runtime volume remains the safe digital 1..30 control. */
    (void)write_reg(REG_LOUT1, 58U);
    (void)write_reg(REG_ROUT1, 0x0100U | 58U);
    (void)write_reg(REG_LOUT2, 58U);
    (void)write_reg(REG_ROUT2, 0x0100U | 58U);
}

void WM8978_SetPlaying(uint32_t playing)
{
    g_playing = playing != 0U;
}

uint32_t WM8978_IsPlaying(void) { return g_playing; }
uint8_t WM8978_GetVolume(void) { return g_volume; }

uint32_t WM8978_BufferedFrames(void)
{
    uint32_t write = g_pcm_write, read = g_pcm_read;
    return write >= read ? write - read : PCM_RING_FRAMES - read + write;
}

uint32_t WM8978_PlayedSeconds(void)
{
    uint32_t seconds;
    __disable_irq();
    seconds = g_played_seconds;
    __enable_irq();
    return seconds;
}

uint32_t WM8978_HasPlayedAudio(void)
{
    return g_played_seconds != 0U || g_played_frame_remainder != 0U;
}

void WM8978_ClearPCM(void)
{
    __disable_irq();
    g_pcm_read = g_pcm_write = 0U;
    g_played_seconds = 0U;
    g_played_frame_remainder = 0U;
    g_audio_started_once = 0U;
    __enable_irq();
}

void WM8978_ResetPlayedTime(void)
{
    __disable_irq();
    g_played_seconds = 0U;
    g_played_frame_remainder = 0U;
    __enable_irq();
}

uint32_t WM8978_QueuePCM(const int16_t *samples, uint32_t frames,
                         uint32_t channels)
{
    uint32_t written = 0U;
    if (channels == 0U || channels > 2U) return 0U;
    while (written < frames) {
        uint32_t write = g_pcm_write;
        uint32_t next = write + 1U;
        if (next == PCM_RING_FRAMES) next = 0U;
        if (next == g_pcm_read) break;
        int16_t left = samples[written * channels];
        int16_t right = channels == 2U ? samples[written * 2U + 1U] : left;
        /* The module's speaker connector is one BTL/mono output.  Average in
         * 32 bits to avoid both signed overflow and analogue mixer clipping,
         * then feed the same sample to each DAC; ROUT2 is inverted by R43. */
        int16_t mono = (int16_t)(((int32_t)left + (int32_t)right) / 2);
        left = mono;
        right = mono;
        g_pcm[write * 2U] = left;
        g_pcm[write * 2U + 1U] = right;
        __DMB();
        g_pcm_write = next;
        written++;
    }
    /* Use a large initial prebuffer for 320 kbit/s startup.  After a rare
     * underrun, restart with about 0.18 s instead of waiting another full
     * second; the network continues rebuilding the ring in parallel. */
    uint32_t restart_frames = g_audio_started_once ? 12288U : 81920U;
    if (WM8978_BufferedFrames() >= restart_frames &&
        (SAI1_Block_A->IMR & SAI_xIMR_FREQIE) == 0U) {
        SAI1_Block_A->IMR = SAI_xIMR_FREQIE;
        g_audio_started_once = 1U;
    }
    return written;
}

void SAI1_IRQHandler(void)
{
    if ((SAI1_Block_A->SR & SAI_xSR_FREQ) != 0U) {
        /* Fill all available FIFO positions in one interrupt.  FLVL=4 means
         * full; the FIFO contains eight 32-bit words (four stereo frames). */
        while (((SAI1_Block_A->SR & SAI_xSR_FLVL_Msk) >>
                SAI_xSR_FLVL_Pos) != 4U) {
            if (g_tx_slot == 0U) {
                g_tx_left = g_tx_right = 0;
                if (g_playing && g_pcm_read != g_pcm_write) {
                    uint32_t read = g_pcm_read;
                    g_tx_left = g_pcm[read * 2U];
                    g_tx_right = g_pcm[read * 2U + 1U];
                    read++;
                    if (read == PCM_RING_FRAMES) read = 0U;
                    g_pcm_read = read;
                    /* User volume 25 preserves the calibrated output level;
                     * range 1..30 provides fine, predictable adjustment. */
                    int32_t left = ((int32_t)g_tx_left * g_volume) / 25;
                    int32_t right = ((int32_t)g_tx_right * g_volume) / 25;
                    if (left > 32767) left = 32767;
                    if (left < -32768) left = -32768;
                    if (right > 32767) right = 32767;
                    if (right < -32768) right = -32768;
                    g_tx_left = (int16_t)left;
                    g_tx_right = (int16_t)right;
                    if (++g_played_frame_remainder >= g_sample_rate) {
                        g_played_frame_remainder -= g_sample_rate;
                        g_played_seconds++;
                    }
                } else if (g_playing) {
                    /* Feed silence for the rest of this FIFO, then let the
                     * producer rebuild the prebuffer before restarting. */
                    SAI1_Block_A->IMR &= ~SAI_xIMR_FREQIE;
                }
                SAI1_Block_A->DR = (uint32_t)(uint16_t)g_tx_left;
                g_tx_slot = 1U;
            } else {
                SAI1_Block_A->DR = (uint32_t)(uint16_t)g_tx_right;
                g_tx_slot = 0U;
            }
        }
        if ((SAI1_Block_A->SR & SAI_xSR_OVRUDR) != 0U)
            SAI1_Block_A->CLRFR = SAI_xCLRFR_COVRUDR;
    }
}
