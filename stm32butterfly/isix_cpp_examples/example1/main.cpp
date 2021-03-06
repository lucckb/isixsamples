#include <isix.h>
#include <stm32lib.h>
#include <stm32system.h>
#include "blinker.hpp"
#include "ledkey.hpp"
#include <isix/arch/irq.h>


namespace {
 
//! Configure ADC prescaler to 8
const unsigned long RCC_CFGR_ADCPRE_8 = (3<<14);

//! HSE oscilator control
const unsigned long RCC_CR_HSI_ON = (1<<0);
//Number of isix threads


 
/** Cortex stm32 System setup
 * Clock and flash configuration for selected rate
 */
void uc_periph_setup()
{
	//Configure CLK clock
    RCC->CR &= ~RCC_CR_HSEON;
    //Disable Bypass
    RCC->CR &= ~RCC_CR_HSEBYP;
    //Enable high speed oscilator
    RCC->CR  |= RCC_CR_HSEON;
    //Wait for setup HSE
    while(1)
    {
        if(RCC->CR & RCC_CR_HSERDY) break;
    }
    //Configure flash: Prefetch enable and 1 wait state
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;

    /* Configure system clocks ALL clocks freq 16MHz
     * PLL x 2
     */
     RCC->CFGR = RCC_CFGR_ADCPRE_8  | RCC_CFGR_PPRE1_1 | RCC_PLLMul_9 |
				RCC_CFGR_PPRE2_1 | RCC_CFGR_PLLSRC;
     RCC->CFGR2 = RCC_CFGR2_PREDIV1_DIV3;

    // At end disable HSI oscilator for power reduction
    RCC->CR &= ~RCC_CR_HSI_ON;

    //Enable PLL
    RCC->CR |= RCC_CR_PLLON;

    //Wait for PLL sync
    while(1)
    {
	    if(RCC->CR & RCC_CR_PLLRDY)
	    {
	        //PLL as system clock
	        RCC->CFGR |=  RCC_CFGR_SW_PLL;
	        break;
	    }
    }
    //Setup NVIC vector at begin of flash
    SCB->VTOR = NVIC_VectTab_FLASH;
}
 
extern "C"
{

 
//! This function is called just before call global constructors
void _external_startup(void)
{

	//Initialize system perhipheral
	uc_periph_setup();

	//1 bit for preemtion priority
	isix_set_irq_priority_group( isix_cortexm_group_pri7 );

	//Initialize isix
	isix_init( CONFIG_HCLK_HZ );

}
 
} /* extern C */

 
}

 
//App main entry point
int main()
{
	//The blinker class
	static app::ledblink led_blinker;
	//The ledkey class
	static app::ledkey led_key;
	
	led_blinker.start();
	led_key.start();
	//Start the isix scheduler
	isix::start_scheduler();
}

 

