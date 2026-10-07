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
     */

#include "stm32f411xe.h"
#include <stdio.h>
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
int _write(int fd, char *buf, int len){
  (void)fd;
  for (int i =0; i<len; i++){
    while(!(USART2->SR & USART_SR_TXE));
    USART2->DR = (uint8_t)buf[i];
  }
  return len;
};
uint8_t uart1_getc(void)
{
    while (!(USART1->SR & USART_SR_RXNE));
    return USART1->DR;
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
    
crc8_init();

while (1) {
    uint8_t pkt[47];


    if (uart1_getc() != 0x54) continue;
    if (uart1_getc() != 0x2C) continue;
    pkt[0] = 0x54;
    pkt[1] = 0x2C;

    //
    for (int i = 2; i < 47; i++)
        pkt[i] = uart1_getc();

    // 
    if (crc8(pkt, 46) != pkt[46]) {
        printf("CRC FAIL\n");
        continue;
    }

    // 
    uint16_t speed = u16le(&pkt[2]);   
    uint16_t start = u16le(&pkt[4]);    
    uint16_t end   = u16le(&pkt[42]);   

    // 
    if (start > 1000) continue;

    printf("spd=%u start=%u.%02u end=%u.%02u | ",
           speed, start / 100, start % 100, end / 100, end % 100);

    for (int k = 0; k < 12; k++) {
        uint16_t dist = u16le(&pkt[6 + 3 * k]);  
        printf("%u ", dist);
    }
    printf("\n");
}
}