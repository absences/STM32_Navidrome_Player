#include "stm32h743xx.h"
#include "wifi_config.h"
#include "navidrome_config.h"
#include "wm8978_playback.h"
#include "sdram.h"
#include "minimp3.h"
#include <stdint.h>

#define OLED_ADDR 0x3CU
#define OLED_COLUMN_OFFSET 2U /* SH1106 has a 132-column RAM. */
#define SCL_PIN 8U
#define SDA_PIN 9U
#define ESP_BAUD 115200U
#define KEY_CONFIRM_PIN 4U /* PC4 */
#define KEY_PUSH_PIN 5U    /* PC5 */
#define ENCODER_A_PIN 4U   /* PA4 */
#define ENCODER_B_PIN 5U   /* PA5 */
#define KEY_BACK_PIN 6U    /* PA6 */
#define WM8978_DIAGNOSTIC 0U
#define OLED_ONLY_DIAGNOSTIC 0U
#define SDRAM_CACHE_ENABLE 1U

static uint8_t framebuffer[128U * 8U];
#define UART_RX_SIZE 32768U
static volatile uint8_t uart_rx[UART_RX_SIZE];
static volatile uint32_t uart_rx_read;
static volatile uint32_t uart_rx_write;
static uint32_t uart_kernel_clock = 64000000U;

static char *append_text(char *destination, const char *source);
static char *append_unsigned(char *destination, uint32_t value);
static uint32_t text_length(const char *text_value);
static uint32_t esp_command(const char *command, uint32_t timeout_ms);
static void ui_show(const char *network, const char *event);
static void ui_play(const char *title, const char *next_title,
                    uint32_t elapsed, uint32_t duration, uint32_t title_skip);
static void ui_volume(uint8_t volume);
static void ui_buffering(const char *title);
static uint32_t utf8_next(const char **text_value);

static void delay_cycles(uint32_t cycles)
{
    uint32_t start = DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT - start) < cycles) { }
}

static void delay_us(uint32_t us)
{
    delay_cycles((SystemCoreClock / 1000000U) * us);
}

static void system_clock_200mhz(void)
{
    /* Conservative clock that needs no regulator/boost transition:
     * HSI64 / 8 * 50 / 2 = 200 MHz CPU, AHB=100 MHz, APB=50 MHz. */
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY_Msk) |
                 FLASH_ACR_LATENCY_4WS;
    while ((FLASH->ACR & FLASH_ACR_LATENCY_Msk) != FLASH_ACR_LATENCY_4WS) { }

    RCC->CR &= ~RCC_CR_PLL1ON;
    while ((RCC->CR & RCC_CR_PLL1RDY) != 0U) { }
    RCC->PLLCKSELR = (RCC->PLLCKSELR &
                     ~(RCC_PLLCKSELR_PLLSRC_Msk | RCC_PLLCKSELR_DIVM1_Msk)) |
                    RCC_PLLCKSELR_PLLSRC_HSI |
                    (8UL << RCC_PLLCKSELR_DIVM1_Pos);
    RCC->PLL1DIVR = (49UL << RCC_PLL1DIVR_N1_Pos) |
                    (1UL << RCC_PLL1DIVR_P1_Pos) |
                    (3UL << RCC_PLL1DIVR_Q1_Pos) |
                    (1UL << RCC_PLL1DIVR_R1_Pos);
    RCC->PLL1FRACR = 0U;
    RCC->PLLCFGR = (RCC->PLLCFGR &
                   ~(RCC_PLLCFGR_PLL1RGE_Msk | RCC_PLLCFGR_PLL1VCOSEL)) |
                  RCC_PLLCFGR_PLL1RGE_3 | RCC_PLLCFGR_DIVP1EN;

    RCC->D1CFGR = RCC_D1CFGR_D1CPRE_DIV1 | RCC_D1CFGR_HPRE_DIV2 |
                  RCC_D1CFGR_D1PPRE_DIV2;
    RCC->D2CFGR = RCC_D2CFGR_D2PPRE1_DIV2 | RCC_D2CFGR_D2PPRE2_DIV2;
    RCC->D3CFGR = RCC_D3CFGR_D3PPRE_DIV2;
    RCC->CR |= RCC_CR_PLL1ON;
    while ((RCC->CR & RCC_CR_PLL1RDY) == 0U) { }
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | RCC_CFGR_SW_PLL1;
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL1) { }
    SystemCoreClockUpdate();
    uart_kernel_clock = 50000000U;
}

static uint32_t elapsed_ms(uint32_t start, uint32_t timeout_ms)
{
    return (uint32_t)(DWT->CYCCNT - start) >=
           ((SystemCoreClock / 1000U) * timeout_ms);
}

static void uart3_init(void)
{
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN;
    RCC->APB1LENR |= RCC_APB1LENR_USART3EN;
    (void)RCC->APB1LENR;

    /* PB10 = USART3_TX, PB11 = USART3_RX, alternate function 7. */
    GPIOB->MODER &= ~((3UL << (10U * 2U)) | (3UL << (11U * 2U)));
    GPIOB->MODER |= (2UL << (10U * 2U)) | (2UL << (11U * 2U));
    GPIOB->AFR[1] &= ~((0xFUL << ((10U - 8U) * 4U)) |
                       (0xFUL << ((11U - 8U) * 4U)));
    GPIOB->AFR[1] |= (7UL << ((10U - 8U) * 4U)) |
                     (7UL << ((11U - 8U) * 4U));
    GPIOB->OSPEEDR |= (3UL << (10U * 2U)) | (3UL << (11U * 2U));
    GPIOB->PUPDR &= ~((3UL << (10U * 2U)) | (3UL << (11U * 2U)));
    GPIOB->PUPDR |= (1UL << (11U * 2U));

    USART3->CR1 = 0U;
    USART3->CR2 = 0U;
    USART3->CR3 = 0U;
    USART3->PRESC = 0U;
    USART3->BRR = (uart_kernel_clock + (ESP_BAUD / 2U)) / ESP_BAUD;
    USART3->ICR = 0xFFFFFFFFU;
    USART3->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
    while ((USART3->ISR & (USART_ISR_TEACK | USART_ISR_REACK)) !=
           (USART_ISR_TEACK | USART_ISR_REACK)) { }
    USART3->CR1 |= USART_CR1_RXNEIE_RXFNEIE;
    NVIC_SetPriority(USART3_IRQn, 3U);
    NVIC_EnableIRQ(USART3_IRQn);
}

static void uart3_set_baud(uint32_t baud)
{
    while ((USART3->ISR & USART_ISR_TC) == 0U) { }
    USART3->CR1 &= ~USART_CR1_UE;
    USART3->BRR = (uart_kernel_clock + (baud / 2U)) / baud;
    USART3->CR1 |= USART_CR1_UE;
    while ((USART3->ISR & (USART_ISR_TEACK | USART_ISR_REACK)) !=
           (USART_ISR_TEACK | USART_ISR_REACK)) { }
}

void USART3_IRQHandler(void)
{
    uint32_t status = USART3->ISR;
    if ((status & USART_ISR_RXNE_RXFNE) != 0U) {
        uint32_t next = (uart_rx_write + 1U) & (UART_RX_SIZE - 1U);
        uint8_t value = (uint8_t)USART3->RDR;
        if (next != uart_rx_read) {
            uart_rx[uart_rx_write] = value;
            uart_rx_write = next;
        }
    }
    if ((status & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE)) != 0U)
        USART3->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF;
}

static uint32_t uart3_read(uint8_t *value)
{
    if (uart_rx_read == uart_rx_write) return 0U;
    *value = uart_rx[uart_rx_read];
    uart_rx_read = (uart_rx_read + 1U) & (UART_RX_SIZE - 1U);
    return 1U;
}

static void controls_init(void)
{
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOCEN;
    (void)RCC->AHB4ENR;

    GPIOC->MODER &= ~((3UL << (KEY_CONFIRM_PIN * 2U)) |
                      (3UL << (KEY_PUSH_PIN * 2U)));
    GPIOC->PUPDR &= ~((3UL << (KEY_CONFIRM_PIN * 2U)) |
                      (3UL << (KEY_PUSH_PIN * 2U)));
    GPIOC->PUPDR |= (1UL << (KEY_CONFIRM_PIN * 2U)) |
                    (1UL << (KEY_PUSH_PIN * 2U));

    GPIOA->MODER &= ~((3UL << (ENCODER_A_PIN * 2U)) |
                      (3UL << (ENCODER_B_PIN * 2U)) |
                      (3UL << (KEY_BACK_PIN * 2U)));
    GPIOA->PUPDR &= ~((3UL << (ENCODER_A_PIN * 2U)) |
                      (3UL << (ENCODER_B_PIN * 2U)) |
                      (3UL << (KEY_BACK_PIN * 2U)));
    GPIOA->PUPDR |= (1UL << (ENCODER_A_PIN * 2U)) |
                    (1UL << (ENCODER_B_PIN * 2U)) |
                    (1UL << (KEY_BACK_PIN * 2U));
}

static uint32_t control_is_low(GPIO_TypeDef *port, uint32_t pin)
{
    return ((port->IDR & (1UL << pin)) == 0U);
}

static uint32_t encoder_state(void)
{
    return (control_is_low(GPIOA, ENCODER_A_PIN) << 1U) |
           control_is_low(GPIOA, ENCODER_B_PIN);
}

static volatile int32_t encoder_delta;
static volatile uint32_t track_action_pending;
static uint32_t encoder_previous;
static int32_t encoder_phase;

void SysTick_Handler(void)
{
    static const int8_t quadrature[16] = {
         0, -1, 1, 0, 1, 0, 0, -1,
        -1,  0, 0, 1, 0, 1, -1, 0
    };
    uint32_t current = encoder_state();
    static uint8_t next_history = 0xFFU, back_history = 0xFFU;
    static uint32_t next_latched, back_latched;
    encoder_phase += quadrature[(encoder_previous << 2U) | current];
    encoder_previous = current;
    if (encoder_phase >= 4) {
        if (encoder_delta < 30) encoder_delta++;
        encoder_phase = 0;
    } else if (encoder_phase <= -4) {
        if (encoder_delta > -30) encoder_delta--;
        encoder_phase = 0;
    }
    next_history = (uint8_t)((next_history << 1U) |
                   !control_is_low(GPIOC, KEY_CONFIRM_PIN));
    back_history = (uint8_t)((back_history << 1U) |
                   !control_is_low(GPIOA, KEY_BACK_PIN));
    if (next_history == 0U && !next_latched) {
        next_latched = 1U;
        track_action_pending = 2U;
    } else if (next_history == 0xFFU) next_latched = 0U;
    if (back_history == 0U && !back_latched) {
        back_latched = 1U;
        track_action_pending = 3U;
    } else if (back_history == 0xFFU) back_latched = 0U;
}

static void encoder_timer_init(void)
{
    encoder_previous = encoder_state();
    encoder_phase = 0;
    encoder_delta = 0;
    track_action_pending = 0U;
    (void)SysTick_Config(SystemCoreClock / 1000U);
    NVIC_SetPriority(SysTick_IRQn, 10U);
}

static uint32_t track_action_take(void)
{
    __disable_irq();
    uint32_t action = track_action_pending;
    track_action_pending = 0U;
    __enable_irq();
    return action;
}

static int32_t encoder_take_delta(void)
{
    __disable_irq();
    int32_t delta = encoder_delta;
    encoder_delta = 0;
    __enable_irq();
    return delta;
}

static void uart3_write(const char *s)
{
    while (*s != '\0') {
        while ((USART3->ISR & USART_ISR_TXE_TXFNF) == 0U) { }
        USART3->TDR = (uint8_t)*s++;
    }
    while ((USART3->ISR & USART_ISR_TC) == 0U) { }
}

static void uart3_discard_rx(void)
{
    uart_rx_read = uart_rx_write;
    USART3->ICR = USART_ICR_ORECF | USART_ICR_FECF |
                  USART_ICR_NECF | USART_ICR_PECF;
}

static uint32_t esp_wait_text(const char *expected, uint32_t timeout_ms)
{
    uint32_t matched = 0U;
    uint32_t expected_length = 0U;
    uint32_t start = DWT->CYCCNT;

    while (expected[expected_length] != '\0') expected_length++;
    if (expected_length == 0U) return 1U;

    while (!elapsed_ms(start, timeout_ms)) {
        uint8_t byte_value;
        if (uart3_read(&byte_value)) {
            char value = (char)byte_value;
            if (value == expected[matched]) {
                if (++matched == expected_length) return 1U;
            } else {
                matched = (value == expected[0]) ? 1U : 0U;
            }
        }
    }
    return 0U;
}

static uint32_t esp_send_http(const char *path)
{
    char command[96];
    char request[512];
    char *end = command;
    end = append_text(end, "AT+CIPSTART=\"TCP\",\"");
    end = append_text(end, NAVIDROME_HOST);
    end = append_text(end, "\",");
    end = append_unsigned(end, NAVIDROME_PORT);
    *end = '\0';
    if (!esp_command(command, 10000U)) return 0U;

    end = request;
    end = append_text(end, "GET ");
    end = append_text(end, path);
    /* HTTP/1.0 keeps live transcoding close-delimited.  HTTP/1.1 would use
     * chunked transfer coding and inject hexadecimal chunk lengths between
     * MP3 frames, requiring another parser on the MCU. */
    end = append_text(end, " HTTP/1.0\r\nHost: ");
    end = append_text(end, NAVIDROME_HOST);
    end = append_text(end, "\r\nConnection: close\r\n\r\n");
    *end = '\0';
    end = command;
    end = append_text(end, "AT+CIPSEND=");
    end = append_unsigned(end, text_length(request));
    *end = '\0';
    uart3_discard_rx();
    uart3_write(command); uart3_write("\r\n");
    if (!esp_wait_text(">", 5000U)) return 0U;
    uart3_write(request);
    return esp_wait_text("SEND OK", 5000U);
}

static void nav_path(char *path, const char *method, const char *extra)
{
    char *end = path;
    end = append_text(end, "/rest/"); end = append_text(end, method);
    end = append_text(end, ".view?u="); end = append_text(end, NAVIDROME_USER);
    end = append_text(end, "&p="); end = append_text(end, NAVIDROME_PASSWORD);
    end = append_text(end, "&v=1.16.1&c=HScreamSTM32&f=json");
    end = append_text(end, extra); *end = '\0';
}

static uint32_t nav_random_song(char *song_id, char *title,
                                uint32_t *duration)
{
    char path[320];
    nav_path(path, "getRandomSongs", "&size=1");
    if (!esp_send_http(path)) return 0U;
    static const char id_key[] = "\"id\":\"";
    static const char title_key[] = "\"title\":\"";
    static const char artist_key[] = "\"artist\":\"";
    static const char duration_key[] = "\"duration\":";
    uint32_t id_match = 0U, title_match = 0U, artist_match = 0U;
    uint32_t duration_match = 0U;
    uint32_t id_length = 0U, title_length = 0U, artist_length = 0U;
    uint32_t capture = 0U, got_id = 0U, got_title = 0U, got_artist = 0U;
    char artist[95];
    *duration = 0U; title[0] = '\0';
    uint32_t start = DWT->CYCCNT;
    while (!elapsed_ms(start, 12000U)) {
        uint8_t value;
        if (!uart3_read(&value)) continue;
        if (capture == 1U) {
            if (value == '"') { song_id[id_length] = '\0'; got_id = 1U; capture = 0U; }
            else if (id_length < 63U) song_id[id_length++] = (char)value;
        } else if (capture == 2U) {
            if (value == '"') { title[title_length] = '\0'; got_title = 1U; capture = 0U; }
            else if (title_length < 94U) title[title_length++] = (char)value;
        } else if (capture == 3U) {
            if (value >= '0' && value <= '9')
                *duration = *duration * 10U + value - '0';
            else if (*duration != 0U && got_id && got_title && got_artist) {
                title[title_length++] = '-';
                for (uint32_t i = 0U; i < artist_length; ++i)
                    title[title_length++] = artist[i];
                title[title_length] = '\0';
                return 1U;
            }
        } else if (capture == 4U) {
            if (value == '"') {
                artist[artist_length] = '\0';
                got_artist = 1U;
                capture = 0U;
            } else if (artist_length < 94U) artist[artist_length++] = (char)value;
        } else {
            if (value == (uint8_t)id_key[id_match]) {
                if (++id_match == sizeof(id_key) - 1U) { capture = 1U; id_match = 0U; }
            } else id_match = value == (uint8_t)id_key[0] ? 1U : 0U;
            if (value == (uint8_t)title_key[title_match]) {
                if (++title_match == sizeof(title_key) - 1U) { capture = 2U; title_match = 0U; }
            } else title_match = value == (uint8_t)title_key[0] ? 1U : 0U;
            if (value == (uint8_t)artist_key[artist_match]) {
                if (++artist_match == sizeof(artist_key) - 1U) { capture = 4U; artist_match = 0U; }
            } else artist_match = value == (uint8_t)artist_key[0] ? 1U : 0U;
            if (value == (uint8_t)duration_key[duration_match]) {
                if (++duration_match == sizeof(duration_key) - 1U) { capture = 3U; duration_match = 0U; }
            } else duration_match = value == (uint8_t)duration_key[0] ? 1U : 0U;
        }
    }
    return 0U;
}

typedef struct {
    uint32_t state;
    uint32_t length;
    uint32_t remaining;
} ipd_parser_t;

static uint32_t ipd_payload(ipd_parser_t *parser, uint8_t input,
                            uint8_t *output)
{
    static const char prefix[] = "+IPD,";
    if (parser->state < 5U) {
        if (input == (uint8_t)prefix[parser->state]) parser->state++;
        else parser->state = input == '+' ? 1U : 0U;
    } else if (parser->state == 5U) {
        if (input >= '0' && input <= '9')
            parser->length = parser->length * 10U + input - '0';
        else if (input == ':') {
            parser->remaining = parser->length;
            parser->length = 0U;
            parser->state = 6U;
        } else parser->state = 0U;
    } else {
        *output = input;
        if (--parser->remaining == 0U) parser->state = 0U;
        return 1U;
    }
    return 0U;
}

static uint32_t nav_start_stream(const char *song_id)
{
    char path[384];
    char extra[128];
    char *end = extra;
    end = append_text(end, "&id="); end = append_text(end, song_id);
    end = append_text(end, "&format=mp3&maxBitRate=320"); *end = '\0';
    nav_path(path, "stream", extra);
    (void)esp_command("AT+CIPCLOSE", 1000U);
    if (!esp_command("AT+CIPRECVMODE=1", 2000U)) return 0U;
    return esp_send_http(path);
}

static uint32_t esp_passive_read(uint8_t *destination, uint32_t requested,
                                 uint32_t timeout_ms)
{
    static const char prefix[] = "+CIPRECVDATA:";
    char command[40];
    char *end = append_text(command, "AT+CIPRECVDATA=");
    end = append_unsigned(end, requested); *end = '\0';
    uart3_discard_rx();
    uart3_write(command); uart3_write("\r\n");

    uint32_t match = 0U, actual = 0U, header_done = 0U, received = 0U;
    uint32_t start = DWT->CYCCNT;
    while (!elapsed_ms(start, timeout_ms)) {
        uint8_t value;
        if (!uart3_read(&value)) continue;
        if (!header_done) {
            if (match < sizeof(prefix) - 1U) {
                if (value == (uint8_t)prefix[match]) match++;
                else match = value == '+' ? 1U : 0U;
            } else if (value >= '0' && value <= '9') {
                actual = actual * 10U + value - '0';
            } else if (value == ',') {
                header_done = 1U;
                if (actual > requested) actual = requested;
                if (actual == 0U) return 0U;
            } else {
                match = 0U; actual = 0U;
            }
        } else {
            destination[received++] = value;
            if (received == actual) return received;
        }
    }
    return 0U;
}

static uint32_t play_stream_cached(const char *song_id, const char *title,
                                   const char *next_id, const char *next_title,
                                   uint32_t duration,
                                   uint32_t timeout_ms)
{
    #define SONG_CACHE_BYTES (14UL * 1024UL * 1024UL)
    #define PLAY_START_BYTES (256UL * 1024UL)
    static char cached_id[2][64];
    static uint32_t cached_length[2];
    static uint32_t replacement_slot;
    static int16_t pcm[MINIMP3_MAX_SAMPLES_PER_FRAME];
    uint8_t *cache[2] = {
        (uint8_t *)SDRAM_BASE_ADDR,
        (uint8_t *)(SDRAM_BASE_ADDR + SONG_CACHE_BYTES)
    };
    uint32_t current_slot = 2U;
    for (uint32_t slot = 0U; slot < 2U; ++slot) {
        uint32_t i = 0U;
        while (cached_id[slot][i] == song_id[i] && song_id[i] != '\0') i++;
        if (cached_id[slot][i] == '\0' && song_id[i] == '\0' &&
            cached_length[slot] != 0U) current_slot = slot;
    }
    if (current_slot == 2U || WM8978_BufferedFrames() == 0U)
        ui_buffering(title);
    uint32_t queued_action = track_action_take();
    if (queued_action != 0U) return queued_action;
    uint32_t current_downloading = 0U;
    uint32_t current_empty_reads = 0U;
    if (current_slot == 2U) {
        current_slot = replacement_slot;
        replacement_slot ^= 1U;
        cached_id[current_slot][0] = '\0';
        cached_length[current_slot] = 0U;
        if (!nav_start_stream(song_id)) return 0U;
        current_downloading = 1U;
        while (cached_length[current_slot] < PLAY_START_BYTES &&
               cached_length[current_slot] < SONG_CACHE_BYTES) {
            uint32_t room = SONG_CACHE_BYTES - cached_length[current_slot];
            uint32_t request = room > 4096U ? 4096U : room;
            uint32_t obtained = esp_passive_read(
                cache[current_slot] + cached_length[current_slot], request, 1000U);
            if (obtained != 0U) {
                cached_length[current_slot] += obtained;
                current_empty_reads = 0U;
            } else if (++current_empty_reads >= 5U) {
                current_downloading = 0U;
                break;
            }
            queued_action = track_action_take();
            if (queued_action != 0U) {
                (void)esp_command("AT+CIPCLOSE", 1000U);
                cached_length[current_slot] = 0U;
                return queued_action;
            }
        }
        if (!current_downloading) {
            (void)esp_command("AT+CIPCLOSE", 1000U);
            (void)esp_command("AT+CIPRECVMODE=0", 2000U);
        }
        if (cached_length[current_slot] < 1024U) return 0U;
        uint32_t i = 0U;
        do { cached_id[current_slot][i] = song_id[i]; }
        while (song_id[i++] != '\0' && i < 64U);
    }

    uint32_t next_slot = current_slot ^ 1U;
    cached_id[next_slot][0] = '\0';
    cached_length[next_slot] = 0U;
    uint32_t prefetch_started = 0U, prefetch_done = next_id[0] == '\0';
    uint32_t prefetch_empty = 0U;
    uint32_t compressed_position = 0U;
    uint32_t track_sample_rate = 0U;
    mp3dec_t decoder;
    uint8_t push_history = 0xFFU;
    uint32_t push_latched = 0U;
    uint32_t volume_overlay = 0U, volume_overlay_start = 0U;
    uint32_t title_skip = 0U, title_scroll_start = DWT->CYCCNT;
    int32_t title_scroll_direction = 1;
    mp3dec_init(&decoder);
    if (WM8978_BufferedFrames() == 0U) WM8978_ClearPCM();
    else WM8978_ResetPlayedTime();
    uint32_t shown_second = 0xFFFFFFFFU;
    (void)timeout_ms;
    while (1) {
        uint32_t played_second = WM8978_PlayedSeconds();
        if (volume_overlay && elapsed_ms(volume_overlay_start, 1000U)) {
            volume_overlay = 0U;
            shown_second = 0xFFFFFFFFU;
        }
        uint32_t title_scroll_due = 0U;
        if (!volume_overlay && elapsed_ms(title_scroll_start, 700U)) {
            title_scroll_start = DWT->CYCCNT;
            const char *measure = title;
            uint32_t remaining_width = 0U;
            while (*measure != '\0') {
                uint32_t cp = utf8_next(&measure);
                remaining_width += (cp >= 0x4E00U && cp <= 0x9FFFU) ? 16U : 6U;
            }
            if (remaining_width > 128U) {
                if (title_scroll_direction > 0) title_skip++;
                else if (title_skip != 0U) title_skip--;
                measure = title;
                for (uint32_t i = 0U; i < title_skip && *measure != '\0'; ++i)
                    (void)utf8_next(&measure);
                remaining_width = 0U;
                while (*measure != '\0') {
                    uint32_t cp = utf8_next(&measure);
                    remaining_width += (cp >= 0x4E00U && cp <= 0x9FFFU) ? 16U : 6U;
                }
                if (remaining_width <= 128U) title_scroll_direction = -1;
                if (title_skip == 0U) title_scroll_direction = 1;
                title_scroll_due = 1U;
            }
        }
        if (!volume_overlay && WM8978_HasPlayedAudio() &&
            (played_second != shown_second || title_scroll_due)) {
            ui_play(title, next_title, played_second, duration, title_skip);
            shown_second = played_second;
        }
        int32_t encoder_change = encoder_take_delta();
        if (encoder_change != 0) {
            int32_t volume = (int32_t)WM8978_GetVolume() + encoder_change;
            if (volume < 1) volume = 1;
            if (volume > 30) volume = 30;
            WM8978_SetVolume((uint8_t)volume);
            ui_volume(WM8978_GetVolume());
            volume_overlay = 1U;
            volume_overlay_start = DWT->CYCCNT;
        }
        push_history = (uint8_t)((push_history << 1U) |
                       !control_is_low(GPIOC, KEY_PUSH_PIN));
        if (push_history == 0U && !push_latched) {
            push_latched = 1U;
            WM8978_SetPlaying(!WM8978_IsPlaying());
            ui_play(title, next_title, WM8978_PlayedSeconds(), duration,
                    title_skip);
            shown_second = WM8978_PlayedSeconds();
            volume_overlay = 0U;
        } else if (push_history == 0xFFU) push_latched = 0U;
        queued_action = track_action_take();
        if (queued_action == 2U) {
            ui_buffering(title);
            (void)esp_command("AT+CIPCLOSE", 1000U);
            (void)esp_command("AT+CIPRECVMODE=0", 2000U);
            uart3_discard_rx();
            WM8978_ClearPCM();
            cached_length[next_slot] = 0U;
            if (current_downloading) {
                cached_length[current_slot] = 0U;
                cached_id[current_slot][0] = '\0';
            }
            return 2U;
        }
        if (queued_action == 3U) {
            ui_buffering(title);
            (void)esp_command("AT+CIPCLOSE", 1000U);
            (void)esp_command("AT+CIPRECVMODE=0", 2000U);
            uart3_discard_rx();
            WM8978_ClearPCM();
            cached_length[next_slot] = 0U;
            if (current_downloading) {
                cached_length[current_slot] = 0U;
                cached_id[current_slot][0] = '\0';
            }
            return 3U;
        }
        uint32_t available_compressed =
            cached_length[current_slot] - compressed_position;
        if (available_compressed != 0U &&
            (available_compressed >= 16384U || !current_downloading) &&
            WM8978_BufferedFrames() < 300000U) {
            mp3dec_frame_info_t info;
            uint32_t decode_bytes =
                available_compressed > 16384U ? 16384U : available_compressed;
            int samples = mp3dec_decode_frame(
                &decoder, cache[current_slot] + compressed_position,
                (int)decode_bytes, pcm, &info);
            if (info.frame_bytes > 0) {
                compressed_position += (uint32_t)info.frame_bytes;
                if (samples > 0 && (info.hz == 44100 || info.hz == 48000) &&
                    (info.channels == 1 || info.channels == 2)) {
                    if (track_sample_rate == 0U)
                        track_sample_rate = (uint32_t)info.hz;
                    if ((uint32_t)info.hz != track_sample_rate) continue;
                    WM8978_SetSampleRate(track_sample_rate);
                    (void)WM8978_QueuePCM(pcm, (uint32_t)samples,
                                          (uint32_t)info.channels);
                }
                continue;
            }
            /* A zero-sized frame at the live edge normally means that the
             * final MP3 frame is split across two ESP reads.  Preserve it
             * until more bytes arrive; skipping one byte here repeatedly
             * drops audio and makes playback sound fast before it stalls. */
            if (!current_downloading) {
                compressed_position++;
                continue;
            }
        }
        if (current_downloading &&
            (WM8978_BufferedFrames() >= 88200U ||
             compressed_position >= cached_length[current_slot] ||
             cached_length[current_slot] - compressed_position < 16384U)) {
            uint32_t room = SONG_CACHE_BYTES - cached_length[current_slot];
            if (room == 0U) current_downloading = 0U;
            else {
                uint32_t request = room > 4096U ? 4096U : room;
                uint32_t obtained = esp_passive_read(
                    cache[current_slot] + cached_length[current_slot],
                    request, 1000U);
                if (obtained != 0U) {
                    cached_length[current_slot] += obtained;
                    current_empty_reads = 0U;
                } else if (++current_empty_reads >= 5U) {
                    current_downloading = 0U;
                }
            }
            if (!current_downloading) {
                (void)esp_command("AT+CIPCLOSE", 1000U);
                (void)esp_command("AT+CIPRECVMODE=0", 2000U);
            }
            continue;
        }
        if (!current_downloading && !prefetch_started && !prefetch_done &&
            WM8978_BufferedFrames() >= 132300U) {
            prefetch_started = nav_start_stream(next_id);
            if (!prefetch_started) prefetch_done = 1U;
        }
        if (prefetch_started && !prefetch_done &&
            (WM8978_BufferedFrames() >= 88200U ||
             compressed_position >= cached_length[current_slot])) {
            uint32_t room = SONG_CACHE_BYTES - cached_length[next_slot];
            if (room == 0U) prefetch_done = 1U;
            else {
                uint32_t request = room > 4096U ? 4096U : room;
                uint32_t obtained = esp_passive_read(
                    cache[next_slot] + cached_length[next_slot], request, 1000U);
                if (obtained != 0U) {
                    cached_length[next_slot] += obtained;
                    prefetch_empty = 0U;
                } else if (++prefetch_empty >= 5U) prefetch_done = 1U;
            }
            if (prefetch_done) {
                (void)esp_command("AT+CIPCLOSE", 1000U);
                (void)esp_command("AT+CIPRECVMODE=0", 2000U);
                if (cached_length[next_slot] >= 1024U) {
                    uint32_t i = 0U;
                    do { cached_id[next_slot][i] = next_id[i]; }
                    while (next_id[i++] != '\0' && i < 64U);
                }
            }
        }
        if (!current_downloading &&
            compressed_position >= cached_length[current_slot] &&
            prefetch_done) break;
    }
    (void)esp_command("AT+CIPCLOSE", 1000U);
    (void)esp_command("AT+CIPRECVMODE=0", 2000U);
    return 1U;
}

/* Known-good real-time path.  Decode while downloading instead of treating the
 * SDRAM song cache as a clock source; this keeps the I2S rate tied strictly to
 * each MP3 frame's sample rate and avoids fast playback followed by a stall. */
static uint32_t play_stream(const char *song_id, const char *title,
                            const char *next_title, uint32_t duration,
                            uint32_t timeout_ms)
{
    static uint8_t compressed[16384];
    static int16_t pcm[MINIMP3_MAX_SAMPLES_PER_FRAME];
    uint32_t compressed_size = 0U;
    mp3dec_t decoder;
    uint8_t push_history = 0xFFU;
    uint32_t push_latched = 0U;
    uint32_t volume_overlay = 0U, volume_overlay_start = 0U;
    uint32_t title_skip = 0U, title_scroll_start = DWT->CYCCNT;
    int32_t title_scroll_direction = 1;
    mp3dec_init(&decoder);
    WM8978_ClearPCM();
    ui_buffering(title);
    uint32_t queued_action = track_action_take();
    if (queued_action != 0U) return queued_action;
    if (!nav_start_stream(song_id)) return 0U;
    uint32_t last_data = DWT->CYCCNT;
    uint32_t shown_second = 0xFFFFFFFFU;
    while (1) {
        if (!WM8978_IsPlaying())
            last_data = DWT->CYCCNT;
        else if (elapsed_ms(last_data, timeout_ms))
            break;
        uint32_t played_second = WM8978_PlayedSeconds();
        if (volume_overlay && elapsed_ms(volume_overlay_start, 1000U)) {
            volume_overlay = 0U;
            shown_second = 0xFFFFFFFFU;
        }
        uint32_t title_scroll_due = 0U;
        if (!volume_overlay && elapsed_ms(title_scroll_start, 700U)) {
            title_scroll_start = DWT->CYCCNT;
            const char *measure = title;
            uint32_t remaining_width = 0U;
            while (*measure != '\0') {
                uint32_t cp = utf8_next(&measure);
                remaining_width += (cp >= 0x4E00U && cp <= 0x9FFFU) ? 16U : 6U;
            }
            if (remaining_width > 128U) {
                if (title_scroll_direction > 0) title_skip++;
                else if (title_skip != 0U) title_skip--;
                measure = title;
                for (uint32_t i = 0U; i < title_skip && *measure != '\0'; ++i)
                    (void)utf8_next(&measure);
                remaining_width = 0U;
                while (*measure != '\0') {
                    uint32_t cp = utf8_next(&measure);
                    remaining_width += (cp >= 0x4E00U && cp <= 0x9FFFU) ? 16U : 6U;
                }
                if (remaining_width <= 128U) title_scroll_direction = -1;
                if (title_skip == 0U) title_scroll_direction = 1;
                title_scroll_due = 1U;
            }
        }
        if (!volume_overlay && WM8978_HasPlayedAudio() &&
            (played_second != shown_second || title_scroll_due)) {
            ui_play(title, next_title, played_second, duration, title_skip);
            shown_second = played_second;
        }
        int32_t encoder_change = encoder_take_delta();
        if (encoder_change != 0) {
            int32_t volume = (int32_t)WM8978_GetVolume() + encoder_change;
            if (volume < 1) volume = 1;
            if (volume > 30) volume = 30;
            WM8978_SetVolume((uint8_t)volume);
            ui_volume(WM8978_GetVolume());
            volume_overlay = 1U;
            volume_overlay_start = DWT->CYCCNT;
        }
        push_history = (uint8_t)((push_history << 1U) |
                       !control_is_low(GPIOC, KEY_PUSH_PIN));
        if (push_history == 0U && !push_latched) {
            push_latched = 1U;
            WM8978_SetPlaying(!WM8978_IsPlaying());
            ui_play(title, next_title, WM8978_PlayedSeconds(), duration,
                    title_skip);
            shown_second = WM8978_PlayedSeconds();
            volume_overlay = 0U;
        } else if (push_history == 0xFFU) push_latched = 0U;
        queued_action = track_action_take();
        if (queued_action == 2U || queued_action == 3U) {
            ui_buffering(title);
            (void)esp_command("AT+CIPCLOSE", 1000U);
            (void)esp_command("AT+CIPRECVMODE=0", 2000U);
            uart3_discard_rx();
            return queued_action;
        }
        if (compressed_size < 1024U &&
            compressed_size <= sizeof(compressed) - 4096U) {
            uint32_t obtained = esp_passive_read(compressed + compressed_size,
                                                 4096U, 1000U);
            if (obtained != 0U) {
                compressed_size += obtained;
                last_data = DWT->CYCCNT;
            }
        }
        if (compressed_size >= 1024U && WM8978_BufferedFrames() < 96000U) {
            mp3dec_frame_info_t info;
            int samples = mp3dec_decode_frame(&decoder, compressed,
                                              (int)compressed_size, pcm, &info);
            if (info.frame_bytes > 0) {
                uint32_t consumed = (uint32_t)info.frame_bytes;
                for (uint32_t i = consumed; i < compressed_size; ++i)
                    compressed[i - consumed] = compressed[i];
                compressed_size -= consumed;
                if (samples > 0) {
                    WM8978_SetSampleRate((uint32_t)info.hz);
                    (void)WM8978_QueuePCM(pcm, (uint32_t)samples,
                                          (uint32_t)info.channels);
                }
            }
        }
    }
    (void)esp_command("AT+CIPCLOSE", 1000U);
    (void)esp_command("AT+CIPRECVMODE=0", 2000U);
    return 1U;
}

static uint32_t esp_command(const char *command, uint32_t timeout_ms)
{
    uart3_discard_rx();
    uart3_write(command);
    uart3_write("\r\n");
    return esp_wait_text("OK", timeout_ms);
}

static uint32_t esp_at_test(void)
{
    for (uint32_t attempt = 0U; attempt < 5U; ++attempt) {
        if (esp_command("AT", 1000U)) return 1U;
        delay_us(200000U);
    }
    /* Recover a module left at 230400/460800 by an earlier MCU reset, then
     * return both ends to the known 115200 startup setting. */
    static const uint32_t recovery_baud[] = {230400U, 460800U};
    for (uint32_t i = 0U; i < 2U; ++i) {
        uart3_set_baud(recovery_baud[i]);
        uart3_discard_rx();
        if (esp_command("AT", 1500U)) {
            /* UART_CUR takes effect before its trailing OK can be received at
             * the old baud, so transmit it without treating that lost OK as
             * failure, switch the MCU, and verify at the new rate. */
            uart3_discard_rx();
            uart3_write("AT+UART_CUR=115200,8,1,0,0\r\n");
            delay_us(200000U);
            uart3_set_baud(115200U);
            uart3_discard_rx();
            if (esp_command("AT", 1500U)) return 1U;
        }
    }
    uart3_set_baud(115200U);
    uart3_discard_rx();
    return 0U;
}

static char *append_text(char *destination, const char *source)
{
    while (*source != '\0') *destination++ = *source++;
    return destination;
}

static char *append_unsigned(char *destination, uint32_t value)
{
    char reversed[10];
    uint32_t count = 0U;
    do {
        reversed[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U);
    while (count != 0U) *destination++ = reversed[--count];
    return destination;
}

static uint32_t text_length(const char *text_value)
{
    uint32_t length = 0U;
    while (text_value[length] != '\0') length++;
    return length;
}

static uint32_t esp_join_wifi(void)
{
    char command[160];
    char *end = command;

    (void)esp_command("ATE0", 1000U);
    if (!esp_command("AT+CWMODE=1", 2000U)) return 0U;

    end = append_text(end, "AT+CWJAP=\"");
    end = append_text(end, WIFI_SSID);
    end = append_text(end, "\",\"");
    end = append_text(end, WIFI_PASSWORD);
    end = append_text(end, "\"");
    *end = '\0';

    uint32_t joined = 0U;
    for (uint32_t attempt = 0U; attempt < 3U && !joined; ++attempt) {
        joined = esp_command(command, 20000U);
        if (!joined) delay_us(500000U);
    }
    if (!joined) return 0U;
    /* Switch both ends to 921600 after joining Wi-Fi.  This doubles the UART
     * payload margin for high-bitrate MP3 while retaining reliable 115200
     * communication during ESP-AT startup. */
    if (esp_command("AT+UART_CUR=921600,8,1,0,0", 2000U)) {
        delay_us(100000U);
        uart3_set_baud(921600U);
        uart3_discard_rx();
    }
    (void)esp_command("AT+CIPRECVMODE=0", 2000U);
    (void)esp_command("AT+CIPDINFO=0", 2000U);
    return esp_command("AT+CIFSR", 3000U);
}

static uint32_t esp_navidrome_ping(void)
{
    char command[96];
    char request[384];
    char *end;

    (void)esp_command("AT+CIPCLOSE", 1000U);
    if (!esp_command("AT+CIPMUX=0", 2000U)) return 0U;

    end = command;
    end = append_text(end, "AT+CIPSTART=\"TCP\",\"");
    end = append_text(end, NAVIDROME_HOST);
    end = append_text(end, "\",");
    end = append_unsigned(end, NAVIDROME_PORT);
    *end = '\0';
    if (!esp_command(command, 10000U)) return 0U;

    end = request;
    end = append_text(end, "GET /rest/ping.view?u=");
    end = append_text(end, NAVIDROME_USER);
    end = append_text(end, "&p=");
    end = append_text(end, NAVIDROME_PASSWORD);
    end = append_text(end, "&v=1.16.1&c=HScreamSTM32&f=json HTTP/1.1\r\nHost: ");
    end = append_text(end, NAVIDROME_HOST);
    end = append_text(end, "\r\nConnection: close\r\n\r\n");
    *end = '\0';

    end = command;
    end = append_text(end, "AT+CIPSEND=");
    end = append_unsigned(end, text_length(request));
    *end = '\0';
    uart3_discard_rx();
    uart3_write(command);
    uart3_write("\r\n");
    if (!esp_wait_text(">", 5000U)) {
        (void)esp_command("AT+CIPCLOSE", 2000U);
        return 0U;
    }

    uart3_write(request);
    uint32_t ok = esp_wait_text("subsonic-response", 10000U);
    (void)esp_command("AT+CIPCLOSE", 2000U);
    return ok;
}

static void gpio_release(uint32_t pin) { GPIOB->BSRR = 1UL << pin; }
static void gpio_low(uint32_t pin) { GPIOB->BSRR = 1UL << (pin + 16U); }
static uint32_t gpio_read(uint32_t pin) { return (GPIOB->IDR >> pin) & 1U; }

static void i2c_delay(void) { delay_us(3U); }

static void i2c_start(void)
{
    gpio_release(SDA_PIN); gpio_release(SCL_PIN); i2c_delay();
    gpio_low(SDA_PIN); i2c_delay();
    gpio_low(SCL_PIN);
}

static void i2c_stop(void)
{
    gpio_low(SDA_PIN); gpio_release(SCL_PIN); i2c_delay();
    gpio_release(SDA_PIN); i2c_delay();
}

static uint32_t i2c_write_byte(uint8_t value)
{
    for (uint32_t i = 0U; i < 8U; ++i) {
        if ((value & 0x80U) != 0U) gpio_release(SDA_PIN); else gpio_low(SDA_PIN);
        i2c_delay(); gpio_release(SCL_PIN); i2c_delay(); gpio_low(SCL_PIN);
        value <<= 1U;
    }
    gpio_release(SDA_PIN); i2c_delay(); gpio_release(SCL_PIN); i2c_delay();
    uint32_t ack = (gpio_read(SDA_PIN) == 0U);
    gpio_low(SCL_PIN);
    return ack;
}

static uint32_t oled_begin(uint8_t control)
{
    i2c_start();
    if (!i2c_write_byte((uint8_t)(OLED_ADDR << 1U))) { i2c_stop(); return 0U; }
    if (!i2c_write_byte(control)) { i2c_stop(); return 0U; }
    return 1U;
}

static void oled_cmd(uint8_t command)
{
    if (oled_begin(0x00U)) i2c_write_byte(command);
    i2c_stop();
}

static void oled_init(void)
{
    static const uint8_t init[] = {
        0xAE, 0x20, 0x00, 0xB0, 0xC8, 0x00, 0x10, 0x40,
        0x81, 0x7F, 0xA1, 0xA6, 0xA8, 0x3F, 0xA4, 0xD3,
        0x00, 0xD5, 0x80, 0xD9, 0xF1, 0xDA, 0x12, 0xDB,
        0x40, 0x8D, 0x14, 0xAF
    };
    delay_us(100000U);
    for (uint32_t i = 0U; i < sizeof(init); ++i) oled_cmd(init[i]);
}

static void oled_pixel(uint32_t x, uint32_t y)
{
    if (x < 128U && y < 64U) framebuffer[x + (y / 8U) * 128U] |= (uint8_t)(1U << (y & 7U));
}

static void glyph(uint32_t x, uint32_t y, char c)
{
    static const uint8_t blank[5] = {0,0,0,0,0};
    static const uint8_t A[5] = {0x7E,0x11,0x11,0x11,0x7E};
    static const uint8_t B[5] = {0x7F,0x49,0x49,0x49,0x36};
    static const uint8_t C[5] = {0x3E,0x41,0x41,0x41,0x22};
    static const uint8_t D[5] = {0x7F,0x41,0x41,0x22,0x1C};
    static const uint8_t E[5] = {0x7F,0x49,0x49,0x49,0x41};
    static const uint8_t F[5] = {0x7F,0x09,0x09,0x09,0x01};
    static const uint8_t G[5] = {0x3E,0x41,0x49,0x49,0x3A};
    static const uint8_t H[5] = {0x7F,0x08,0x08,0x08,0x7F};
    static const uint8_t I[5] = {0x41,0x41,0x7F,0x41,0x41};
    static const uint8_t K[5] = {0x7F,0x08,0x14,0x22,0x41};
    static const uint8_t L[5] = {0x7F,0x40,0x40,0x40,0x40};
    static const uint8_t M[5] = {0x7F,0x02,0x0C,0x02,0x7F};
    static const uint8_t N[5] = {0x7F,0x04,0x08,0x10,0x7F};
    static const uint8_t O[5] = {0x3E,0x41,0x41,0x41,0x3E};
    static const uint8_t P[5] = {0x7F,0x09,0x09,0x09,0x06};
    static const uint8_t R[5] = {0x7F,0x09,0x19,0x29,0x46};
    static const uint8_t S[5] = {0x26,0x49,0x49,0x49,0x32};
    static const uint8_t T[5] = {0x01,0x01,0x7F,0x01,0x01};
    static const uint8_t U[5] = {0x3F,0x40,0x40,0x40,0x3F};
    static const uint8_t V[5] = {0x1F,0x20,0x40,0x20,0x1F};
    static const uint8_t W[5] = {0x7F,0x20,0x18,0x20,0x7F};
    static const uint8_t X[5] = {0x63,0x14,0x08,0x14,0x63};
    static const uint8_t Y[5] = {0x07,0x08,0x70,0x08,0x07};
    static const uint8_t colon[5] = {0x00,0x36,0x36,0x00,0x00};
    static const uint8_t slash[5] = {0x20,0x10,0x08,0x04,0x02};
    static const uint8_t question[5] = {0x02,0x01,0x51,0x09,0x06};
    static const uint8_t digits[10][5] = {
        {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00},
        {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31},
        {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39},
        {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
        {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E}
    };
    const uint8_t *pattern = blank;
    if (c >= '0' && c <= '9') pattern = digits[(uint32_t)(c - '0')];
    else switch (c) {
    case 'A': pattern = A; break; case 'B': pattern = B; break;
    case 'C': pattern = C; break; case 'D': pattern = D; break;
    case 'E': pattern = E; break;
    case 'F': pattern = F; break; case 'G': pattern = G; break;
    case 'H': pattern = H; break; case 'I': pattern = I; break;
    case 'K': pattern = K; break; case 'M': pattern = M; break;
    case 'L': pattern = L; break; case 'N': pattern = N; break;
    case 'P': pattern = P; break; case 'R': pattern = R; break;
    case 'O': pattern = O; break; case 'S': pattern = S; break;
    case 'T': pattern = T; break;
    case 'U': pattern = U; break; case 'V': pattern = V; break;
    case 'W': pattern = W; break; case 'X': pattern = X; break;
    case 'Y': pattern = Y; break;
    case ':': pattern = colon; break; case '/': pattern = slash; break;
    case '?': pattern = question; break; default: break;
    }
    for (uint32_t col = 0U; col < 5U; ++col)
        for (uint32_t row = 0U; row < 7U; ++row)
            if ((pattern[col] >> row) & 1U) oled_pixel(x + col, y + row);
}

static void text(uint32_t x, uint32_t y, const char *s)
{
    while (*s != '\0') { glyph(x, y, *s++); x += 6U; }
}

extern const uint8_t _binary_cjk16_bin_start[];

static uint32_t utf8_next(const char **text_value)
{
    const uint8_t *s = (const uint8_t *)*text_value;
    uint32_t codepoint;
    if (s[0] < 0x80U) { codepoint = s[0]; *text_value += 1; }
    else if ((s[0] & 0xE0U) == 0xC0U && (s[1] & 0xC0U) == 0x80U) {
        codepoint = ((uint32_t)(s[0] & 0x1FU) << 6U) | (s[1] & 0x3FU);
        *text_value += 2;
    } else if ((s[0] & 0xF0U) == 0xE0U &&
               (s[1] & 0xC0U) == 0x80U && (s[2] & 0xC0U) == 0x80U) {
        codepoint = ((uint32_t)(s[0] & 0x0FU) << 12U) |
                    ((uint32_t)(s[1] & 0x3FU) << 6U) | (s[2] & 0x3FU);
        *text_value += 3;
    } else { codepoint = '?'; *text_value += 1; }
    return codepoint;
}

static void title_utf8_skip(uint32_t x, uint32_t y, const char *title,
                            uint32_t skip)
{
    while (skip-- != 0U && *title != '\0') (void)utf8_next(&title);
    while (*title != '\0' && x < 128U) {
        uint32_t codepoint = utf8_next(&title);
        if (codepoint >= 0x4E00U && codepoint <= 0x9FFFU) {
            const uint8_t *bitmap = _binary_cjk16_bin_start +
                                    (codepoint - 0x4E00U) * 32U;
            for (uint32_t row = 0U; row < 16U; ++row) {
                uint16_t bits = ((uint16_t)bitmap[row * 2U] << 8U) |
                                bitmap[row * 2U + 1U];
                for (uint32_t col = 0U; col < 16U && x + col < 128U; ++col)
                    if ((bits & (0x8000U >> col)) != 0U)
                        oled_pixel(x + col, y + row);
            }
            x += 16U;
        } else {
            glyph(x, y + 4U, codepoint < 128U ? (char)codepoint : '?');
            x += 6U;
        }
    }
}

static void title_utf8(uint32_t x, uint32_t y, const char *title)
{
    title_utf8_skip(x, y, title, 0U);
}

static void oled_flush(void)
{
    /* Page addressing works on SH1106 and SSD1306. SH1106 provides 132 RAM
     * columns, while the glass uses the centered 128 columns (offset 2). */
    for (uint32_t page = 0U; page < 8U; ++page) {
        uint32_t column = OLED_COLUMN_OFFSET;
        oled_cmd((uint8_t)(0xB0U | page));
        oled_cmd((uint8_t)(column & 0x0FU));
        oled_cmd((uint8_t)(0x10U | (column >> 4U)));
        if (oled_begin(0x40U)) {
            for (uint32_t x = 0U; x < 128U; ++x)
                i2c_write_byte(framebuffer[(page * 128U) + x]);
        }
        i2c_stop();
    }
}

static void ui_show(const char *network, const char *event)
{
    for (uint32_t i = 0U; i < sizeof(framebuffer); ++i) framebuffer[i] = 0U;
    text(38U, 2U, "PC MUSIC");
    text(32U, 22U, network);
    text(26U, 42U, event);
    oled_flush();
}

static void ui_play(const char *title, const char *next_title,
                    uint32_t elapsed, uint32_t duration, uint32_t title_skip)
{
    char time_text[16];
    char *end = time_text;
    uint32_t em = elapsed / 60U, es = elapsed % 60U;
    uint32_t dm = duration / 60U, ds = duration % 60U;
    if (em > 99U) em = 99U;
    if (dm > 99U) dm = 99U;
    *end++ = (char)('0' + em / 10U); *end++ = (char)('0' + em % 10U);
    *end++ = ':'; *end++ = (char)('0' + es / 10U); *end++ = (char)('0' + es % 10U);
    *end++ = '/'; *end++ = (char)('0' + dm / 10U); *end++ = (char)('0' + dm % 10U);
    *end++ = ':'; *end++ = (char)('0' + ds / 10U); *end++ = (char)('0' + ds % 10U);
    *end = '\0';
    for (uint32_t i = 0U; i < sizeof(framebuffer); ++i) framebuffer[i] = 0U;
    title_utf8_skip(0U, 2U, title, title_skip);
    /* Next-track marker and prefetched title. */
    for (uint32_t x = 0U; x < 7U; ++x) {
        oled_pixel(2U + x, 27U + x);
        oled_pixel(2U + x, 39U - x);
    }
    title_utf8(12U, 23U, next_title);
    text(2U, 46U, time_text);
    oled_flush();
}

static void ui_volume(uint8_t volume)
{
    char number[4];
    char *end = append_unsigned(number, volume);
    *end = '\0';
    if (volume < 1U) volume = 1U;
    if (volume > 30U) volume = 30U;
    for (uint32_t i = 0U; i < sizeof(framebuffer); ++i) framebuffer[i] = 0U;
    text(38U, 5U, "VOLUME");
    text(55U, 20U, number);

    /* 112 x 14 outline, with a 108-pixel proportional fill. */
    for (uint32_t x = 8U; x < 120U; ++x) {
        oled_pixel(x, 40U);
        oled_pixel(x, 53U);
    }
    for (uint32_t y = 40U; y <= 53U; ++y) {
        oled_pixel(8U, y);
        oled_pixel(119U, y);
    }
    uint32_t filled = ((uint32_t)(volume - 1U) * 108U + 14U) / 29U;
    for (uint32_t x = 10U; x < 10U + filled; ++x)
        for (uint32_t y = 42U; y < 52U; ++y) oled_pixel(x, y);
    oled_flush();
}

static void ui_buffering(const char *title)
{
    for (uint32_t i = 0U; i < sizeof(framebuffer); ++i) framebuffer[i] = 0U;
    title_utf8(0U, 3U, title);
    text(34U, 38U, "BUFFERING");
    oled_flush();
}

static void controls_run(const char *network, uint32_t audio_ok)
{
    if (audio_ok && network[5] == 'O') {
        char current[64] = {0};
        char next[64] = {0};
        char previous[64] = {0};
        char current_title[193] = {0}, next_title[193] = {0};
        char previous_title[193] = {0};
        uint32_t current_duration = 0U, next_duration = 0U;
        uint32_t previous_duration = 0U;
        if (!nav_random_song(current, current_title, &current_duration)) {
            ui_show(network, "NO SONG");
        } else {
            ui_buffering(current_title);
            (void)nav_random_song(next, next_title, &next_duration);
        }
        while (current[0] != '\0') {
            uint32_t action = play_stream(current, current_title,
                                          next_title, current_duration,
                                          15000U);
            if (action == 3U && previous[0] != '\0') {
                char old_current[64];
                char old_current_title[193];
                uint32_t old_current_duration = current_duration;
                uint32_t i = 0U;
                do { old_current[i] = current[i]; } while (current[i++] != '\0');
                i = 0U; do { old_current_title[i] = current_title[i]; }
                while (current_title[i++] != '\0');
                i = 0U; do { current[i] = previous[i]; } while (previous[i++] != '\0');
                i = 0U; do { current_title[i] = previous_title[i]; } while (previous_title[i++] != '\0');
                current_duration = previous_duration;
                i = 0U; do { next[i] = old_current[i]; } while (old_current[i++] != '\0');
                i = 0U; do { next_title[i] = old_current_title[i]; }
                while (old_current_title[i++] != '\0');
                next_duration = old_current_duration;
                previous[0] = '\0';
                previous_title[0] = '\0';
            } else {
                uint32_t i = 0U;
                do { previous[i] = current[i]; } while (current[i++] != '\0');
                i = 0U; do { previous_title[i] = current_title[i]; } while (current_title[i++] != '\0');
                previous_duration = current_duration;
                if (next[0] != '\0') {
                    i = 0U; do { current[i] = next[i]; } while (next[i++] != '\0');
                    i = 0U; do { current_title[i] = next_title[i]; }
                    while (next_title[i++] != '\0');
                    current_duration = next_duration;
                } else if (!nav_random_song(current, current_title,
                                             &current_duration)) {
                    ui_show(network, "NO SONG");
                    break;
                }
                next[0] = '\0'; next_title[0] = '\0'; next_duration = 0U;
                (void)nav_random_song(next, next_title, &next_duration);
            }
        }
    }
    uint8_t confirm_history = 0xFFU;
    uint8_t push_history = 0xFFU;
    uint8_t back_history = 0xFFU;
    uint32_t confirm_latched = 0U;
    uint32_t push_latched = 0U;
    uint32_t back_latched = 0U;

    ui_show(network, audio_ok ? "AUDIO: OK" : "AUDIO: NO");
    while (1) {
        int32_t encoder_change = encoder_take_delta();
        if (encoder_change != 0) {
            int32_t volume = (int32_t)WM8978_GetVolume() + encoder_change;
            if (volume < 1) volume = 1;
            if (volume > 30) volume = 30;
            WM8978_SetVolume((uint8_t)volume);
            ui_volume(WM8978_GetVolume());
            delay_us(1000000U);
            ui_show(network, audio_ok ? "AUDIO: OK" : "AUDIO: NO");
        }

        confirm_history = (uint8_t)((confirm_history << 1U) |
                           !control_is_low(GPIOC, KEY_CONFIRM_PIN));
        push_history = (uint8_t)((push_history << 1U) |
                        !control_is_low(GPIOC, KEY_PUSH_PIN));
        back_history = (uint8_t)((back_history << 1U) |
                        !control_is_low(GPIOA, KEY_BACK_PIN));

        if (confirm_history == 0U && !confirm_latched) {
            confirm_latched = 1U;
            ui_show(network, "TRACK: NEXT");
        } else if (confirm_history == 0xFFU) confirm_latched = 0U;

        if (push_history == 0U && !push_latched) {
            push_latched = 1U;
            uint32_t playing = !WM8978_IsPlaying();
            WM8978_SetPlaying(playing);
            ui_show(network, playing ? "PLAY" : "PAUSE");
        } else if (push_history == 0xFFU) push_latched = 0U;

        if (back_history == 0U && !back_latched) {
            back_latched = 1U;
            ui_show(network, "TRACK: PREV");
        } else if (back_history == 0xFFU) back_latched = 0U;

        delay_us(1000U);
    }
}

static void board_init(void)
{
    SystemCoreClockUpdate();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN;
    (void)RCC->AHB4ENR;
    GPIOB->MODER &= ~((3UL << (SCL_PIN * 2U)) | (3UL << (SDA_PIN * 2U)));
    GPIOB->MODER |= (1UL << (SCL_PIN * 2U)) | (1UL << (SDA_PIN * 2U));
    GPIOB->OTYPER |= (1UL << SCL_PIN) | (1UL << SDA_PIN);
    GPIOB->OSPEEDR |= (3UL << (SCL_PIN * 2U)) | (3UL << (SDA_PIN * 2U));
    GPIOB->PUPDR &= ~((3UL << (SCL_PIN * 2U)) | (3UL << (SDA_PIN * 2U)));
    gpio_release(SCL_PIN); gpio_release(SDA_PIN);
}

static void wm8978_diagnostic_tone(uint32_t audio_ok)
{
    static int16_t tone[512U * 2U];
    uint32_t phase = 0U;
    ui_show(audio_ok ? "WM: OK" : "WM: NO",
            audio_ok ? "TONE" : "CHECK WIRE");
    if (!audio_ok) while (1) { }

    WM8978_SetVolume(48U);
    WM8978_SetPlaying(1U);
    while (1) {
        if (WM8978_BufferedFrames() < 7000U) {
            for (uint32_t frame = 0U; frame < 512U; ++frame) {
                /* 32-bit phase accumulator: 1 kHz at 44.1 kHz. */
                int16_t sample = (phase & 0x80000000UL) ? 6000 : -6000;
                tone[frame * 2U] = sample;
                tone[frame * 2U + 1U] = sample;
                phase += 97391549UL;
            }
            (void)WM8978_QueuePCM(tone, 512U, 2U);
        }
    }
}

int main(void)
{
    system_clock_200mhz();
    board_init();
    controls_init();
    encoder_timer_init();
    uart3_init();
    oled_init();
#if SDRAM_CACHE_ENABLE
    if (!SDRAM_InitAndTest()) {
        ui_show("SDRAM: NO", "CHECK FMC");
        while (1) { }
    }
#endif
#if OLED_ONLY_DIAGNOSTIC
    ui_show("OLED OK", "DISPLAY");
    while (1) { }
#endif
#if WM8978_DIAGNOSTIC
    ui_show("WM TEST", "START");
    delay_us(500000U);
    uint32_t diagnostic_audio_ok = WM8978_PlaybackInit();
    wm8978_diagnostic_tone(diagnostic_audio_ok);
#endif
    text(38U, 8U, "PC MUSIC");
    text(38U, 28U, "ESP: AT");
    oled_flush();

    uint32_t esp_ok = esp_at_test();
    if (esp_ok) {
        /* An MCU reset leaves the ESP running.  Clean its session in place;
         * AT+RST is avoided because some ESP-AT builds return at a stored
         * baud or need a variable boot time, causing a false ESP/NAV failure. */
        (void)esp_command("AT+CIPCLOSE", 1500U);
        (void)esp_command("AT+CIPRECVMODE=0", 1500U);
        (void)esp_command("AT+CIPMUX=0", 1500U);
        uart3_discard_rx();
    }
    for (uint32_t i = 0U; i < sizeof(framebuffer); ++i) framebuffer[i] = 0U;
    text(38U, 8U, "PC MUSIC");
    text(38U, 28U, esp_ok ? "ESP: OK" : "ESP: NO");
    oled_flush();

    uint32_t audio_ok = WM8978_PlaybackInit();
#if WM8978_DIAGNOSTIC
    wm8978_diagnostic_tone(audio_ok);
#endif
    uint32_t wifi_ok = 0U;
    if (esp_ok) {
        delay_us(500000U);
        wifi_ok = esp_join_wifi();
    }
    /* Do not gate playback on a separate one-shot ping.  On ESP-AT the
     * server can close that short connection before the matcher consumes
     * the response, even though subsequent API/stream requests work. */
    controls_run(wifi_ok ? "NAV: OK" : "NAV: NO", audio_ok);
}
