/*
 * LiDAR receive driver — STM32F411RE (NUCLEO-F411RE), bare metal, no HAL.
 * Sensor: LDROBOT STL-19P (LD06/LD19 family), 230400 8N1, 47-byte packets.
 *
 * Data path:
 *   USART1_RX (PA10) -> DMA2 Stream2 Ch4, circular 512 B -> HT/TC ISR
 *   -> dma_getc() -> parser_feed() state machine -> CRC8 -> decode
 *   -> USART2 (PA2, ST-LINK VCP): text stats, or binary point frames (PLOT_STREAM)
 */

#include "stm32f411xe.h"
#include <stdio.h>

/* ============================== Build switches ============================== */

#define DROP_TEST     0     /* 1 = drop one byte every 10 ms to exercise resync      */
#define PLOT_STREAM   1     /* 1 = send binary point frames to the PC at 460800 baud */

/* ================================ Constants ================================= */

#define CPU_HZ        100000000u
#define ONE_SEC       CPU_HZ                /* DWT cycles per second                */
#define WAIT_LOOPS    1000000u              /* bound for hardware-flag waits        */

#define RX_BUF_SIZE   512u                  /* power of two -> wrap with a mask     */
#define RX_BUF_MASK   (RX_BUF_SIZE - 1u)
#define RX_HALF       (RX_BUF_SIZE / 2u)    /* bytes delivered per HT/TC interrupt  */

#define PKT_LEN       47u                   /* header(2) + speed(2) + start(2) +
                                               12 x 3 + end(2) + ts(2) + crc(1)    */
#define PKT_POINTS    12u
#define CRC_POLY      0x4Du

#define TEST_SECONDS  10u

/* ============================== Shared state ================================ */

/* CMSIS expects these; clock is configured in clock_init(), not here. */
uint32_t SystemCoreClock = 16000000u;       /* HSI after reset */
void SystemInit(void) { }

/* DMA receive — written by the ISR */
static uint8_t          rx_buf[RX_BUF_SIZE];
static volatile uint32_t dma_written   = 0;  /* total bytes completed by DMA */
static volatile uint32_t ht_count      = 0;
static volatile uint32_t tc_count      = 0;
static volatile uint32_t dma_err_count = 0;

/* DMA receive — written by main */
static uint32_t parsed            = 0;      /* total bytes consumed by main */
static uint32_t dma_lap_count     = 0;
static uint32_t dma_restart_count = 0;
static uint32_t no_data_count     = 0;
static uint32_t buf_hwm           = 0;      /* max unread bytes observed    */

/* Parser — main only */
enum { WAIT_54, WAIT_2C, COLLECT };
static uint8_t  state = WAIT_54;
static uint32_t idx   = 0;
static uint8_t  pkt[PKT_LEN];
static uint32_t packets_ok     = 0;
static uint32_t crc_fail_count = 0;

/* ISR timing (Phase 5) — written by the ISR */
static volatile uint32_t isr_min = 0xFFFFFFFFu;
static volatile uint32_t isr_max = 0;
static volatile uint32_t isr_sum = 0;
static volatile uint32_t isr_n   = 0;

/* Init health */
static uint32_t init_errors = 0;

static uint8_t crc_table[256];

/* ============================== Basic helpers =============================== */

/* Busy-wait on DWT->CYCCNT. Unsigned subtraction is wrap-safe. */
static void delay_us(uint32_t us)
{
    uint32_t cycles = us * (SystemCoreClock / 1000000u);
    uint32_t start  = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles) { }
}

/* Bounded waits for hardware flags. On timeout: count the error and continue. */
static int wait_set(volatile uint32_t *reg, uint32_t mask)
{
    for (uint32_t n = 0; n < WAIT_LOOPS; n++) {
        if (*reg & mask) return 1;
    }
    init_errors++;
    return 0;
}

static int wait_clear(volatile uint32_t *reg, uint32_t mask)
{
    for (uint32_t n = 0; n < WAIT_LOOPS; n++) {
        if (!(*reg & mask)) return 1;
    }
    init_errors++;
    return 0;
}

static inline uint16_t u16le(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

/* ================================ CRC8 ====================================== */

/* Poly 0x4D, init 0, no reflection, no final XOR. */
static void crc8_init(void)
{
    for (int i = 0; i < 256; i++) {
        uint8_t crc = (uint8_t)i;
        for (int j = 0; j < 8; j++) {
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ CRC_POLY)
                                : (uint8_t)(crc << 1);
        }
        crc_table[i] = crc;
    }
}

static uint8_t crc8(const uint8_t *data, int len)
{
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) {
        crc = crc_table[crc ^ data[i]];
    }
    return crc;
}

/* ============================ Hardware init ================================= */

/*
 * GPIO:
 *   PA2  AF7  USART2_TX  (ST-LINK virtual COM port)
 *   PA5  out  LD2 user LED
 *   PA8  AF0  MCO1       (clock check: PLL / 5 = 20 MHz)
 *   PA10 AF7  USART1_RX  (LiDAR TX), pull-up
 */
static void gpio_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    (void)RCC->AHB1ENR;

    /* PA2: AF7 */
    GPIOA->MODER  &= ~(3u << 4);
    GPIOA->MODER  |=  (2u << 4);
    GPIOA->AFR[0] &= ~(0xFu << 8);
    GPIOA->AFR[0] |=  (7u   << 8);

    /* PA5: general-purpose output */
    GPIOA->MODER  &= ~(3u << 10);
    GPIOA->MODER  |=  (1u << 10);

    /* PA8: AF0, very high speed */
    GPIOA->MODER   &= ~(3u << 16);
    GPIOA->MODER   |=  (2u << 16);
    GPIOA->AFR[1]  &= ~(0xFu << 0);
    GPIOA->OSPEEDR |=  (3u << 16);

    /* PA10: AF7, pull-up (line idles high) */
    GPIOA->MODER  &= ~(3u << 20);
    GPIOA->MODER  |=  (2u << 20);
    GPIOA->AFR[1] &= ~(0xFu << 8);
    GPIOA->AFR[1] |=  (7u   << 8);
    GPIOA->PUPDR  &= ~(3u << 20);
    GPIOA->PUPDR  |=  (1u << 20);
}

/*
 * SYSCLK = 100 MHz from PLL. Order matters: voltage scale and flash wait
 * states must be set before switching to the faster clock.
 * Returns 1 if HSE (8 MHz bypass from ST-LINK MCO) is used, 0 if HSI fallback.
 */
static int clock_init(void)
{
    /* HSE in bypass mode: the ST-LINK drives an 8 MHz clock into PH0. */
    RCC->CR &= ~RCC_CR_HSEON;
    wait_clear(&RCC->CR, RCC_CR_HSERDY);
    RCC->CR |= RCC_CR_HSEBYP;
    RCC->CR |= RCC_CR_HSEON;
    int hse_ok = wait_set(&RCC->CR, RCC_CR_HSERDY);
    if (!hse_ok) {
        RCC->CR &= ~RCC_CR_HSEON;           /* fall back to HSI 16 MHz */
    }

    /* Regulator to scale 1 (required for 100 MHz). Read-back delays the
       first PWR access after enabling its clock (errata ES0287). */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    (void)RCC->APB1ENR;
    PWR->CR |= PWR_CR_VOS;
    /* Flash: 3 wait states (90–100 MHz @ 2.7–3.6 V), prefetch, I/D cache. */
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_3WS;
    FLASH->ACR |= FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;
    for (uint32_t n = 0; n < WAIT_LOOPS &&
         (FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_3WS; n++) { }

    /* PLL: VCO input 2 MHz, VCO 200 MHz, P = /2 -> 100 MHz.
       HSE: 8 / 4 * 100 / 2.   HSI: 16 / 8 * 100 / 2.   P encoding 00 = /2. */
    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN |
                      RCC_PLLCFGR_PLLP | RCC_PLLCFGR_PLLSRC);
    if (hse_ok) {
        RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE
                      | (4u   << RCC_PLLCFGR_PLLM_Pos)
                      | (100u << RCC_PLLCFGR_PLLN_Pos)
                      | (0u   << RCC_PLLCFGR_PLLP_Pos);
    } else {
        RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI
                      | (8u   << RCC_PLLCFGR_PLLM_Pos)
                      | (100u << RCC_PLLCFGR_PLLN_Pos)
                      | (0u   << RCC_PLLCFGR_PLLP_Pos);
    }

    /* Start PLL, wait for lock and for the regulator to reach scale 1. */
    RCC->CR |= RCC_CR_PLLON;
    wait_set(&RCC->CR,   RCC_CR_PLLRDY);
    wait_set(&PWR->CSR,  PWR_CSR_VOSRDY);

    /* Bus prescalers: APB1 = /2 (50 MHz, max 50), APB2 = /1 (100 MHz). */
    RCC->CFGR &= ~(RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    RCC->CFGR |=  RCC_CFGR_PPRE1_DIV2;

    /* Switch SYSCLK to PLL and wait for the hardware to confirm. */
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |=  RCC_CFGR_SW_PLL;
    for (uint32_t n = 0; n < WAIT_LOOPS &&
         (RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL; n++) { }

    /* MCO1 on PA8 = PLL / 5 = 20 MHz, for checking the clock with a scope. */
    RCC->CFGR &= ~(RCC_CFGR_MCO1 | RCC_CFGR_MCO1PRE);
    RCC->CFGR |=  (RCC_CFGR_MCO1 | RCC_CFGR_MCO1PRE);

    SystemCoreClock = CPU_HZ;
    return hse_ok;
}

/* DWT cycle counter: 10 ns resolution at 100 MHz. */
static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

/* USART2 TX only, APB1 = 50 MHz.
   BRR 0x1B2 = 27.125 -> 115200.  BRR 0x6D = 6.8125 -> 458716 (-0.45 %). */
static void usart2_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    (void)RCC->APB1ENR;
#if PLOT_STREAM
    USART2->BRR = 0x6D;
#else
    USART2->BRR = 0x1B2;
#endif
    USART2->CR1 = USART_CR1_TE | USART_CR1_UE;
}

/* USART1, APB2 = 100 MHz. BRR 0x1B2 = 27.125 -> 230400 (0.006 % error).
   RE is enabled later, only after DMA is ready to take bytes. */
static void usart1_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    (void)RCC->APB2ENR;
    USART1->BRR = 0x1B2;
    USART1->CR1 = USART_CR1_UE;
}
/* ============================ printf -> USART2 ============================== */

/* newlib hook. Blocking, LF expanded to CRLF. */
int _write(int fd, char *ptr, int len)
{
    (void)fd;
    for (int i = 0; i < len; i++) {
        if (ptr[i] == '\n') {
            while (!(USART2->SR & USART_SR_TXE)) { }
            USART2->DR = '\r';
        }
        while (!(USART2->SR & USART_SR_TXE)) { }
        USART2->DR = (uint8_t)ptr[i];
    }
    return len;
}

/* ============================== DMA receive ================================= */

/*
 * USART1_RX = DMA2 Stream 2, Channel 4. Peripheral-to-memory, byte transfers,
 * memory increment, circular. HT and TC each mean "256 new bytes".
 * Stream 2 flags live in LISR (read) / LIFCR (write-1-to-clear), bits 16–21.
 */
static void dma_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    (void)RCC->AHB1ENR;

    /* Stream must be disabled before it can be configured. */
    DMA2_Stream2->CR &= ~DMA_SxCR_EN;
    wait_clear(&DMA2_Stream2->CR, DMA_SxCR_EN);

    /* Clear stale flags: FEIF, DMEIF, TEIF, HTIF, TCIF (bit 17 is reserved). */
    DMA2->LIFCR = DMA_LIFCR_CFEIF2 | DMA_LIFCR_CDMEIF2 | DMA_LIFCR_CTEIF2
                | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTCIF2;

    DMA2_Stream2->PAR  = (uint32_t)&USART1->DR;
    DMA2_Stream2->M0AR = (uint32_t)rx_buf;
    DMA2_Stream2->NDTR = RX_BUF_SIZE;

    /* DIR = 00 (P->M), PSIZE = MSIZE = 00 (byte), PINC = 0. */
    DMA2_Stream2->CR = (4u << DMA_SxCR_CHSEL_Pos)
                     | DMA_SxCR_MINC
                     | DMA_SxCR_CIRC
                     | DMA_SxCR_TCIE
                     | DMA_SxCR_HTIE
                     | DMA_SxCR_TEIE;

    NVIC_SetPriority(DMA2_Stream2_IRQn, 1);
    NVIC_EnableIRQ(DMA2_Stream2_IRQn);

    DMA2_Stream2->CR |= DMA_SxCR_EN;
}

/* Start reception: DMA requests on, then receiver on — no byte can overrun
   while nothing is reading DR. */
static void usart1_start_rx(void)
{
    USART1->CR3 |= USART_CR3_DMAR;
    USART1->CR1 |= USART_CR1_RE;
}

/* Kept minimal: clear flag, advance the byte counter. Self-timed with DWT. */
void DMA2_Stream2_IRQHandler(void)
{
    uint32_t t0  = DWT->CYCCNT;
    uint32_t isr = DMA2->LISR;                  /* read once */

    if (isr & DMA_LISR_HTIF2) {
        DMA2->LIFCR  = DMA_LIFCR_CHTIF2;
        dma_written += RX_HALF;
        ht_count++;
    }
    if (isr & DMA_LISR_TCIF2) {
        DMA2->LIFCR  = DMA_LIFCR_CTCIF2;
        dma_written += RX_HALF;
        tc_count++;
    }
    if (isr & DMA_LISR_TEIF2) {
        DMA2->LIFCR = DMA_LIFCR_CTEIF2;         /* hardware also clears EN */
        dma_err_count++;
    }

    uint32_t dt = DWT->CYCCNT - t0;
    if (dt < isr_min) isr_min = dt;
    if (dt > isr_max) isr_max = dt;
    isr_sum += dt;
    isr_n++;
}

/* Re-arm after a transfer error. Writing dma_written from main is safe only
   because the stream is off, so its ISR cannot run. */
static void dma_restart(void)
{
    dma_restart_count++;
    parsed      = 0;
    dma_written = 0;
    dma_init();
}

static void background(void);                   /* test harness, defined below */

/* Blocking read of the next byte. Runs background work while waiting.
   Never hangs: restarts DMA on error, reports 1 s of silence. */
static uint8_t dma_getc(void)
{
    uint32_t wait_start = DWT->CYCCNT;
    uint32_t w;

    while ((w = dma_written) == parsed) {
        background();
        if (!(DMA2_Stream2->CR & DMA_SxCR_EN)) {
            dma_restart();
        }
        if ((DWT->CYCCNT - wait_start) >= ONE_SEC) {
            wait_start += ONE_SEC;
            no_data_count++;
            printf("no data from sensor (%lu s)\n", (unsigned long)no_data_count);
        }
    }

    /* rx_buf is not volatile: the barrier keeps its read after the
       dma_written read, for both the compiler and the CPU. */
    __DMB();

    uint32_t pending = w - parsed;
    if (pending > buf_hwm) buf_hwm = pending;

    if (pending > RX_BUF_SIZE) {                /* DMA overwrote unread data */
        dma_lap_count++;
        parsed = w - RX_BUF_SIZE;
    }

    uint8_t b = rx_buf[parsed & RX_BUF_MASK];
    parsed++;

#if DROP_TEST
    static uint32_t last_drop;
    if ((DWT->CYCCNT - last_drop) >= 1000000u && parsed != w) {
        last_drop = DWT->CYCCNT;
        parsed++;                               /* skip one byte */
    }
#endif
    return b;
}

/* ============================ Packet decoding =============================== */

/* Interpolate 12 angles (0.01° units) between start and end, across 0°.
   Multiply before dividing: one truncation per point, no accumulated error. */
static void compute_angles(uint32_t start, uint32_t end, uint32_t angle[PKT_POINTS])
{
    if (end < start) {
        end += 36000u;
    }
    uint32_t span = end - start;
    for (uint32_t i = 0; i < PKT_POINTS; i++) {
        angle[i] = (start + span * i / (PKT_POINTS - 1u)) % 36000u;
    }
}

#if PLOT_STREAM
/*
 * Binary frame, one per valid packet (52 bytes):
 *   AA 55 | count=12 | 12 x (angle u16 LE, distance u16 LE) | XOR
 * XOR covers count + payload. The PC resyncs on AA 55 and drops frames
 * with a bad XOR, so interleaved printf text is tolerated.
 * TX is blocking: 52 bytes at 460800 = 1.13 ms per frame, ~47 % of main's
 * time at 416 frames/s. Well inside the 13.1 ms DMA service window.
 */
static void uart2_putc(uint8_t c)
{
    while (!(USART2->SR & USART_SR_TXE)) { }
    USART2->DR = c;
}

static void plot_send_packet(void)
{
    uint32_t a[PKT_POINTS];
    compute_angles(u16le(&pkt[4]), u16le(&pkt[42]), a);

    uint8_t x = PKT_POINTS;
    uart2_putc(0xAA);
    uart2_putc(0x55);
    uart2_putc(PKT_POINTS);
    for (uint32_t k = 0; k < PKT_POINTS; k++) {
        uint16_t d = u16le(&pkt[6 + 3 * k]);
        uint8_t f[4] = { (uint8_t)a[k], (uint8_t)(a[k] >> 8),
                         (uint8_t)d,    (uint8_t)(d >> 8) };
        for (int j = 0; j < 4; j++) {
            uart2_putc(f[j]);
            x ^= f[j];
        }
    }
    uart2_putc(x);
}
#endif

/*
 * Byte-at-a-time framer: WAIT_54 -> WAIT_2C -> COLLECT (47 bytes) -> CRC.
 * On CRC failure, bytes 1..46 are replayed so a real header hidden inside
 * the bad packet is not lost. Recursion depth is bounded: 46 replayed bytes
 * cannot complete another 47-byte packet.
 */
static void parser_feed(uint8_t b)
{
    switch (state) {
    case WAIT_54:
        if (b == 0x54) {
            pkt[0] = b;
            state  = WAIT_2C;
        }
        break;

    case WAIT_2C:
        if (b == 0x2C) {
            pkt[1] = b;
            idx    = 2;
            state  = COLLECT;
        } else if (b == 0x54) {
            pkt[0] = b;                         /* may be the real start */
        } else {
            state = WAIT_54;
        }
        break;

    case COLLECT:
        pkt[idx++] = b;
        if (idx == PKT_LEN) {
            idx   = 0;
            state = WAIT_54;                    /* reset before any replay */

            if (crc8(pkt, PKT_LEN - 1) == pkt[PKT_LEN - 1]) {
                packets_ok++;
#if PLOT_STREAM
                plot_send_packet();
#endif
            } else {
                crc_fail_count++;
                uint8_t replay[PKT_LEN - 1];    /* pkt is overwritten by the replay */
                for (uint32_t i = 0; i < PKT_LEN - 1; i++) replay[i] = pkt[i + 1];
                for (uint32_t i = 0; i < PKT_LEN - 1; i++) parser_feed(replay[i]);
            }
        }
        break;
    }
}

/* ======================== Test harness / statistics ========================= */

/*
 * Each row runs TEST_SECONDS under a given CPU load and prints rates.
 *   LOAD_IRQ_OFF: main disables interrupts for busy_us every period_ms.
 *   LOAD_SYSTICK: priority-0 SysTick busy-waits busy_us every period_ms.
 */
enum { LOAD_NONE = 0, LOAD_IRQ_OFF = 1, LOAD_SYSTICK = 2 };

typedef struct {
    const char *name;
    uint8_t     load;
    uint32_t    busy_us;
    uint32_t    period_ms;
} test_t;

static const test_t tests[] = {
    { "baseline",             LOAD_NONE,      0,  0 },
    /* Starvation sweep (Phase 2.3 / 3.4) — uncomment to rerun.
    { "irq-off 200us / 20ms", LOAD_IRQ_OFF, 200, 20 },
    { "irq-off 200us / 10ms", LOAD_IRQ_OFF, 200, 10 },
    { "irq-off 200us /  5ms", LOAD_IRQ_OFF, 200,  5 },
    { "irq-off 200us /  2ms", LOAD_IRQ_OFF, 200,  2 },
    { "irq-off  30us /  2ms", LOAD_IRQ_OFF,  30,  2 },
    { "systick 100us /  1ms", LOAD_SYSTICK, 100,  1 },
    { "systick  30us /  1ms", LOAD_SYSTICK,  30,  1 },
    */
};
#define N_TESTS (sizeof(tests) / sizeof(tests[0]))

static volatile uint32_t systick_busy_us = 0;

void SysTick_Handler(void)
{
    delay_us(systick_busy_us);
}

static uint32_t cur_test = 0;
static uint32_t test_start;
static uint32_t last_load;
static uint32_t snap_ok, snap_crc, snap_lap;
static uint32_t snap_tc, snap_ht, snap_err;
static uint32_t snap_isr_sum, snap_isr_n;

static void start_test(uint32_t i)
{
    const test_t *t = &tests[i];

    SysTick->CTRL = 0;
    if (t->load == LOAD_SYSTICK) {
        systick_busy_us = t->busy_us;
        SysTick_Config(t->period_ms * (CPU_HZ / 1000u));
        NVIC_SetPriority(SysTick_IRQn, 0);      /* after SysTick_Config, which sets 15 */
    }

    snap_ok  = packets_ok;
    snap_crc = crc_fail_count;
    snap_lap = dma_lap_count;
    snap_tc  = tc_count;
    snap_ht  = ht_count;
    snap_err = dma_err_count;

    __disable_irq();                            /* ISR writes these */
    isr_min      = 0xFFFFFFFFu;
    isr_max      = 0;
    snap_isr_sum = isr_sum;
    snap_isr_n   = isr_n;
    __enable_irq();

    buf_hwm    = 0;
    test_start = DWT->CYCCNT;
    last_load  = test_start;
}

static void end_test(uint32_t i)
{
    printf("%-22s pkt/s=%3lu crc/s=%3lu laps=%lu tc=%lu ht=%lu err=%lu\n",
           tests[i].name,
           (unsigned long)((packets_ok     - snap_ok)  / TEST_SECONDS),
           (unsigned long)((crc_fail_count - snap_crc) / TEST_SECONDS),
           (unsigned long)(dma_lap_count   - snap_lap),
           (unsigned long)(tc_count        - snap_tc),
           (unsigned long)(ht_count        - snap_ht),
           (unsigned long)(dma_err_count   - snap_err));

    /* Consistent snapshot of ISR statistics. */
    __disable_irq();
    uint32_t mn  = isr_min;
    uint32_t mx  = isr_max;
    uint32_t sum = isr_sum - snap_isr_sum;
    uint32_t n   = isr_n   - snap_isr_n;
    __enable_irq();

    uint32_t avg = n ? sum / n : 0;
    /* CPU share in milli-percent: ISR cycles / total cycles x 100000. */
    uint32_t cpu_mpct = (uint32_t)(((uint64_t)sum * 100000u) /
                                   ((uint64_t)TEST_SECONDS * ONE_SEC));

    printf("5.1 ISR cycles: min=%lu max=%lu avg=%lu (avg %lu ns) n=%lu (%lu/s)\n",
           (unsigned long)(n ? mn : 0), (unsigned long)mx, (unsigned long)avg,
           (unsigned long)(avg * 10u), (unsigned long)n,
           (unsigned long)(n / TEST_SECONDS));
    printf("5.2 throughput: %lu pkt/s\n",
           (unsigned long)((packets_ok - snap_ok) / TEST_SECONDS));
    printf("5.3 errors: crc=%lu laps=%lu dma_err=%lu restarts=%lu no_data=%lu init_errors=%lu\n",
           (unsigned long)(crc_fail_count - snap_crc),
           (unsigned long)(dma_lap_count  - snap_lap),
           (unsigned long)(dma_err_count  - snap_err),
           (unsigned long)dma_restart_count,
           (unsigned long)no_data_count,
           (unsigned long)init_errors);
    printf("5.4 buffer high-water: %lu / %lu bytes\n",
           (unsigned long)buf_hwm, (unsigned long)RX_BUF_SIZE);
    printf("5.5 CPU in ISR: %lu.%03lu %%\n",
           (unsigned long)(cpu_mpct / 1000u), (unsigned long)(cpu_mpct % 1000u));
}

/* Called from dma_getc() while waiting: applies load, rotates test rows. */
static void background(void)
{
    if (cur_test >= N_TESTS) return;
    const test_t *t = &tests[cur_test];

    uint32_t period = t->period_ms * (CPU_HZ / 1000u);
    if (t->load == LOAD_IRQ_OFF && (DWT->CYCCNT - last_load) >= period) {
        last_load += period;
        __disable_irq();
        delay_us(t->busy_us);
        __enable_irq();
    }

    if ((DWT->CYCCNT - test_start) >= TEST_SECONDS * ONE_SEC) {
        end_test(cur_test);
        cur_test++;
        if (cur_test < N_TESTS) {
            start_test(cur_test);
        } else {
            SysTick->CTRL = 0;
            printf("done\n");
        }
    }
}

/* ================================= Faults =================================== */

/* Reset instead of hanging; boot reports it via RCC_CSR_SFTRSTF. */
void HardFault_Handler(void)
{
    NVIC_SystemReset();
}

/* ================================== Main ==================================== */

static void angle_selftest(void)
{
    uint32_t a[PKT_POINTS];

    compute_angles(119, 905, a);                /* expect 119, 261, ..., 905 */
    printf("angles normal: first=%lu third=%lu last=%lu\n",
           (unsigned long)a[0], (unsigned long)a[2], (unsigned long)a[11]);

    compute_angles(35800, 600, a);              /* expect 35800, ..., 18, ..., 600 */
    printf("angles wrap:   first=%lu mid=%lu last=%lu\n",
           (unsigned long)a[0], (unsigned long)a[3], (unsigned long)a[11]);
}

int main(void)
{
    /* 1. Pins and clocks */
    gpio_init();
    int hse_ok = clock_init();
    dwt_init();

    /* 2. Console */
    usart2_init();
    setvbuf(stdout, NULL, _IONBF, 0);
    printf("boot clk=%s init_errors=%lu\n",
           hse_ok ? "HSE" : "HSI", (unsigned long)init_errors);
    if (RCC->CSR & RCC_CSR_SFTRSTF) {
        printf("last reset: software (fault?)\n");
    }
    RCC->CSR |= RCC_CSR_RMVF;                   /* clear reset flags */

    /* 3. Decoder tables and self-tests */
    crc8_init();
    angle_selftest();

    /* 4. Sensor receive path: USART1 -> DMA */
    usart1_init();
    dma_init();
    usart1_start_rx();

    /* 5. Run */
    printf("test: %u s per row\n", (unsigned)TEST_SECONDS);
    start_test(0);

    while (1) {
        parser_feed(dma_getc());
    }
}