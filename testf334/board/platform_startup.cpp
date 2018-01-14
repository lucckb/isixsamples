/*
 * platform_setup.cpp
 *  Platform initializaton specific code
 *  Created on: 20 lis 2013
 *      Author: lucck
 */
#if 0
#include <stm32system.h>
#include <stm32rcc.h>
#include <stm32pwr.h>
#include <stm32adc.h>
#include <stm32exti.h>
#include <stm32rtc.h>
#include <stm32syscfg.h>
#include <stm32dma.h>
#include <stm32gpio.h>
#endif
#include <config/conf.h>
#include <functional>
#include <isix/arch/irq.h>
#include <isix.h>
#include <boot/arch/arm/cortexm/irq_vectors_table.h>
#include <boot/arch/arm/cortexm/crashinfo.h>

namespace drv {
namespace board {

namespace {


/** STM32 F3 system startup
 * 72MHz for AHB, APB2 36MHz APB1
 */
bool uc_periph_setup()
{
#if 0
	using namespace stm32;
	constexpr auto retries=100000;

	isix_set_irq_vectors_base( &_exceptions_vectors );

    //! Deinitialize RCC
    rcc_deinit();

	//Configure flash latency
	rcc_flash_latency( CONFIG_HCLK_HZ );

	//Setup bus dividers
    rcc_pclk2_config( RCC_HCLK_Div1 );
    rcc_pclk1_config( RCC_HCLK_Div2 );
    rcc_hclk_config( RCC_SYSCLK_Div1 );


    //Enable hse generator
    rcc_hse_config( RCC_HSE_ON );
    if( !rcc_wait_for_hse_startup() ) {
		return false;
    }

    //Configure other periph clocks
    rcc_adc_clk_config( RCC_CFGR2_ADCPRE12_DIV64 );					//ADC 12MHz (Max 14)


    //Enable clocks for all GPIOS
    rcc_ahb_periph_clock_cmd( RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN |
		RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIODEN | RCC_AHBENR_GPIOFEN, true );
	// GPIO configuration
	gpio_config_ext(GPIOA,0x9fff, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP );
	gpio_config_ext(GPIOB,0xffff, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP );
	gpio_config_ext(GPIOC,0x3fff, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP );
	gpio_config_ext(GPIOD,0xffff, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP );
	gpio_config_ext(GPIOF,0xfffc, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP );

    //Enable power domain for SDADC and RTC
    rcc_apb1_periph_clock_cmd( RCC_APB1Periph_PWR, true );
    //Enable DMA controllers
    rcc_ahb_periph_clock_cmd( RCC_AHBPeriph_DMA1|RCC_AHBPeriph_DMA2, true );

    //Configure PLL
    rcc_prediv1_config( RCC_PREDIV1_Div1 );
	rcc_pll_config( RCC_PLLSource_PREDIV1, RCC_CFGR_PLLMULL9 );
	rcc_pll_cmd( true );
	// Wait for PLL startup
	bool ok = false;
	for( auto r=0; r<retries; ++r ) {
		if( !rcc_get_flag_status(RCC_FLAG_PLLRDY) ) {
			ok = true;
			break;
		}
	}
	if( ok ) {
		ok = false;
		rcc_sysclk_config( RCC_SYSCLKSource_PLLCLK );
		// Enable PLL as clock source
		constexpr auto PLL_SRC = 0x08;
		for( auto r=0; r<retries; ++r ) {
			if( rcc_get_sysclk_source() != PLL_SRC ) {
				ok = true;
				break;
			}
		}
	}
	return ok;
#endif
}



//! Application crash called from hard fault
void application_crash( crash_mode type, unsigned long* sp )
{
#ifdef PDEBUG
	cortex_cm3_print_core_regs( type, sp );
#else
	static_cast<void>(type);
	static_cast<void>(sp);
#endif
	for(;;) asm volatile("wfi\n");
}


}	//Unnamed NS

extern "C" {


//! This function is called just before call global constructors
void _external_startup(void)
{
	//SysTimer values
	//Give a chance a JTAG to reset the CPU
	for(unsigned i=0; i<1000000; i++) asm volatile("nop\n");

	//Initialize system perhipheral

	if( uc_periph_setup() ) {
		//1 bit for preemtion priority
		isix_set_irq_priority_group( isix_cortexm_group_pri7 );
		//Initialize isix
		isix::init(CONFIG_HCLK_HZ);
	} else {
		//TODO: Handle failure initialization
		//! Initialization failure
		for(;;);
	}
}


//Crash info interrupt handler
void __attribute__((__interrupt__,naked)) hard_fault_exception_vector(void)
{
	_cm3_hard_hault_entry_fn( application_crash );

}

} /* extern C */

 
}}	//NS drv

