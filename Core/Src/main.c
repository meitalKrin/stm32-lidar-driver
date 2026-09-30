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
uint32_t SystemCoreClock = 16000000;   // HSI after reset; set to 100 MHz in main
void SystemInit(void) { }

int main(void) {
    //wakes up Port A so we can use it
    //RCC_AHB1ENR_GPIOAEN Behind the scenes,
    //  it simply represents 1u << 0 (bit 0), which is the exact switch for Port A.
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    
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

   

   while (1) {
    // BLINK LED TEST
    GPIOA->ODR |= (1u << 5);        
    for (volatile int i = 0; i < 400000; i++);
    GPIOA->ODR &= ~(1u << 5);       
    for (volatile int i = 0; i < 400000; i++);
    }
}