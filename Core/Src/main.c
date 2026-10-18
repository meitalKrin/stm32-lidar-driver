/*******************  importent things to know  ********************/
/* 
     * Bypass Mode: 
     * The ST-LINK programmer on the Nucleo board is already generating a perfect 
     * heartbeat (clock signal). This tells the STM32: "Don't do the hard work of 
     * driving a raw crystal. Just bypass that circuitry and listen to the 
     * ready-made signal coming from next door."
     *
     * Turn on Flash Accelerators to stop the CPU from waiting:
     * 
     * 1. PRFTEN (Prefetch): Assumes code runs top-to-bottom and grabs the 
     *    next line in advance. If the code jumps (like hitting a loop or 
     *    'if' statement), it just throws the guess away and gets the new line.
     * 
     * 2. ICEN & DCEN (Caches): Tiny, super-fast memory that saves a copy of 
     *    recently run code. Since our LED blinks in a repeating while(1) loop, 
     *    the CPU runs it directly from this fast cache instead of the slow Flash.
    
      * Receive Buffer Size Calculation (256 bytes):
      * - 115200 baud ÷ 10 bits/char = 11,520 characters/sec.
      * - A 60-character line takes ~5.2 ms (60 / 11,520).
      * - During that window, 5.2 ms ÷ 43.4 µs ≈ 120 bytes arrive.
      * - Doubled for safety margin = 240 bytes.
       * - Next power of two = 256 bytes.
      */
    
#define RB_SIZE 256u
#define RB_MASK (RB_SIZE - 1u)
#include "stm32f411xe.h"
#include <stdio.h>

/* Phase 2.3 starvation test switches
 * STARVE_MODE 0 : normal run
 * STARVE_MODE 1 : main blocks ALL interrupts for STARVE_US every STARVE_PERIOD_MS
 * STARVE_MODE 2 : SysTick (priority 0, above USART1) busy-waits SYSTICK_BUSY_US every 1 ms
 */
#define STARVE_MODE        0
#define STARVE_US          200u
#define STARVE_PERIOD_MS   10u
#define SYSTICK_BUSY_US    100u

uint8_t  buf[RB_SIZE];
volatile uint32_t head = 0;
volatile uint32_t tail = 0;
volatile uint32_t ore_count = 0;          
volatile uint32_t rb_overflow_count = 0;  
volatile uint32_t fe_count = 0;           
uint32_t packets_ok = 0;                  
uint32_t crc_fail_count = 0;
uint32_t SystemCoreClock = 16000000;     // HSI after reset; set to 100 MHz 
void SystemInit(void) { }
uint8_t crc_table[256];

void crc8_init(void){
 for (int i = 0; i<256;i++){
    uint8_t crc = i;
     for (int j = 0; j < 8; j++){
        if(crc & 0x80){
            crc = (crc << 1) ^ 0x4D;
        }else{
             crc = crc << 1;
        }
       
     }
      crc_table[i] = crc;
 }

}


uint8_t crc8(const uint8_t *data, int len)
{
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) {
        crc = crc_table[crc ^ data[i]];
    }
    return crc;
}

int rb_put( uint8_t b) {
    uint32_t next = (head + 1) & RB_MASK;
    if (next == tail) {
        return 0;         
    }
    buf[head] = b;         
    head = next;        
    return 1;
}
int rb_get( uint8_t *out) {
    if (head == tail) {
        return 0;                     
    }
    *out = buf[tail];                  
    tail = (tail + 1) & RB_MASK;       
    return 1;
}
// turn on interupts 
void USART1_IRQHandler(void) {
    uint32_t sr = USART1->SR;

    if (sr & (USART_SR_RXNE | USART_SR_ORE)) {
        uint8_t b = (uint8_t)USART1->DR;      

        if (sr & USART_SR_ORE) ore_count++;
        if (sr & USART_SR_FE)  fe_count++;
        if (!rb_put(b))        rb_overflow_count++;
    }
}


int _write(int fd, char *ptr, int len) {
    (void)fd;
    for (int i = 0; i < len; i++) {
        if (ptr[i] == '\n') {                      
            while (!(USART2->SR & USART_SR_TXE));
            USART2->DR = '\r';                     
        }
        while (!(USART2->SR & USART_SR_TXE));
        USART2->DR = (uint8_t)ptr[i];
    }
    return len;
}
uint16_t u16le(const uint8_t *p)
{
    return p[0] | (p[1] << 8);  
}
void delay_us(uint32_t us)
{
  uint32_t  cycles = us * (SystemCoreClock/1000000);
  uint32_t start = DWT -> CYCCNT;
  while((DWT -> CYCCNT-start)<cycles);
}

#if STARVE_MODE == 2
void SysTick_Handler(void) {
    delay_us(SYSTICK_BUSY_US);
}
#endif

static const uint32_t ONE_SEC = 100000000u;
static uint32_t last_stats;
static uint32_t last_ok;
#if STARVE_MODE == 1
static uint32_t last_starve;
#endif

static void background(void) {
    if ((DWT->CYCCNT - last_stats) >= ONE_SEC) {
        last_stats += ONE_SEC;
        printf("pkt/s=%lu crc=%lu ore=%lu fe=%lu ovf=%lu\n",
               (unsigned long)(packets_ok - last_ok),
               (unsigned long)crc_fail_count,
               (unsigned long)ore_count,
               (unsigned long)fe_count,
               (unsigned long)rb_overflow_count);
        last_ok = packets_ok;
    }

#if STARVE_MODE == 1
    if ((DWT->CYCCNT - last_starve) >= STARVE_PERIOD_MS * 100000u) {
        last_starve += STARVE_PERIOD_MS * 100000u;
        __disable_irq();
        delay_us(STARVE_US);
        __enable_irq();
    }
#endif
}

uint8_t rb_getc(void) {
    uint8_t b;
    while (!rb_get(&b)) {
        background();
    }
    return b;
}
int main(void) {
    //**start with PINS**//
    //wakes up Port A so we can use it
    //RCC_AHB1ENR_GPIOAEN Behind the scenes,
    //  it simply represents 1u << 0 (bit 0), which is the exact switch for Port A.
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    //Set PA8 to alternate function
    GPIOA->MODER &= ~(3U << 16);
    GPIOA->MODER |=  (2U << 16);
      //Output the PLL clock on PA8
    RCC->CFGR &= ~(RCC_CFGR_MCO1 | RCC_CFGR_MCO1PRE);
    RCC->CFGR |=  (RCC_CFGR_MCO1 | RCC_CFGR_MCO1PRE);
     // PA8 = AF0 (MCO1), very high speed
    GPIOA->MODER   &= ~(3U << 16);
    GPIOA->MODER   |=  (2U << 16);
    GPIOA->AFR[1]  &= ~(0xFU << 0);    // AF0
    GPIOA->OSPEEDR |=  (3U << 16);
    //Set PA2 to alternate function AF7
     GPIOA->MODER &= ~(3U << 4);
     GPIOA->MODER |= (2U << 4);
      GPIOA->AFR[0]  &= ~(0xFU << 8);  
      GPIOA->AFR[0] |= (7U   << 8);
    //set PA10 
     GPIOA->MODER   &= ~(3U << 20);
      GPIOA->MODER  |=  (2U << 20);
      GPIOA->AFR[1] &= ~(0xFU << 8);
      GPIOA->AFR[1] |=  (7U   << 8);

      GPIOA->PUPDR &= ~(3U << 20);
      GPIOA->PUPDR |=  (1U << 20);


    // pin A5 reset wiping whatever job Pin 5 had before and then Output Mode in the second line
    GPIOA->MODER &= ~(3u << 10);
    GPIOA->MODER |=  (1u << 10);
    //Turns OFF (HSE) clock,Waits to confirm the HSE is off,Turns on Bypass Mode , 
    // waits confirms the new external clock heartbeat is stable
    RCC->CR &=  ~RCC_CR_HSEON;
    while (RCC->CR & RCC_CR_HSERDY);
    RCC->CR |= RCC_CR_HSEBYP;
    RCC->CR |=  RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));
    // FOR SAFE 100mhz SPEED UP ,Waking up the Power Controller,
    //Internal voltage regulator ,provide maximum voltage SO CPU core doesn't crash later
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    (void)RCC->APB1ENR;          // (change to power settings the exact microsecond after waking it up, the chip ignores you)errata ES0287 safty
    PWR->CR |= PWR_CR_VOS;      
    //tell CPU every get code from Flash memory,wait for 3 clock cycles
    //Prefetch and Caches (ICEN & DCEN)
    //pauses the program until the chip actively confirms that the 3 Wait States have been successfully applied.
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_3WS;
    FLASH->ACR |= (FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN);
    while ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_3WS);
    //this clears out M N P 
    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM | 
                  RCC_PLLCFGR_PLLN | 
                  RCC_PLLCFGR_PLLP | 
                  RCC_PLLCFGR_PLLSRC);
    //plug clock source and set exactly 100 MHz.
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE |
                (4U   << RCC_PLLCFGR_PLLM_Pos) |
                (100U << RCC_PLLCFGR_PLLN_Pos) |
                (0U   << RCC_PLLCFGR_PLLP_Pos);   // P = /2 (encoded 00)
    //PLL engine ON ,wait for PLLRDY,pauses your program until the hardware confirms the clock is completely stable
    //PLL engine - takes a slow heartbeat from the outside and makes it fast on the inside.
    //pauses the program until the chip confirms the internal voltage has safely reached that maximum level
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));
    while (!(PWR->CSR & PWR_CSR_VOSRDY));
    //wipe the old settings completely clean on APB1 and APB2 
    //Take 100 MHz CPU speed, divide it by 2, and send a safe 50 MHz to APB1
    RCC->CFGR &= ~(RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2); 
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
    //throw the master switch (SW), disconnecting the CPU from the slow clock and officially connecting it to the fast PLL engine
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    //wait for flag confirming that the CPU is now successfully running at 100 MHz.
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
    //Enable the USART2 clock
    RCC->APB1ENR |=(1u << 17);
    (void)RCC->APB1ENR; 
    //Set the baud rate to 115200. 
    USART2->BRR = 0x1B2;
    //Enable TE and UE in CR1.
     USART2->CR1  &= ~(USART_CR1_UE);
     USART2->CR1 |= (1u << 13);
     USART2->CR1  &= ~(USART_CR1_TE);
     USART2->CR1  |= (1u << 3);
     //
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  
    //Clock for USART1 (APB2)
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
  (void)RCC->APB2ENR;
  USART1->BRR = 0x1B2;    
    USART1->CR1 |= USART_CR1_RE | USART_CR1_UE;

    

      SystemCoreClock = 100000000;
      setvbuf(stdout, NULL, _IONBF, 0);
      printf("boot\n");  
      printf("STARVE_MODE=%d\n", STARVE_MODE);
    
crc8_init();


   (void)USART1->SR;                  
    (void)USART1->DR;
    NVIC_SetPriority(USART1_IRQn, 1);
    USART1->CR1 |= USART_CR1_RXNEIE;
    NVIC_EnableIRQ(USART1_IRQn);

#if STARVE_MODE == 2
    SysTick_Config(100000);
    NVIC_SetPriority(SysTick_IRQn, 0);
#endif

    last_stats = DWT->CYCCNT;
    last_ok = 0;
#if STARVE_MODE == 1
    last_starve = DWT->CYCCNT;
#endif

while (1) {
    uint8_t pkt[47];
  

    if (rb_getc() != 0x54) continue;
    if (rb_getc() != 0x2C) continue;
    pkt[0] = 0x54;
    pkt[1] = 0x2C;

    //
    for (int i = 2; i < 47; i++)
        pkt[i] = rb_getc();

    // 
    if (crc8(pkt, 46) != pkt[46]) {
        crc_fail_count++;
        continue;
    }
    packets_ok++;
}
}