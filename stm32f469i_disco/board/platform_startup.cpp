/*
 * platform_setup.cpp
 *  Platform initializaton specific code
 *  Created on: 20 lis 2013
 *      Author: lucck
 */
#include <stm32_ll_rcc.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_system.h>
#include <stm32_ll_gpio.h>
#include <stm32_ll_pwr.h>
#include <config/conf.h>
#include <functional>
#include <isix/arch/irq.h>
#include <isix.h>
#include <boot/arch/arm/cortexm/irq_vectors_table.h>
#include <boot/arch/arm/cortexm/crashinfo.h>
#include <periph/drivers/memory/sdram.hpp>

namespace drv {
namespace board {
namespace {


/** STM32F469 system startup
 */
bool uc_periph_setup()
{
	constexpr auto retries=100000;
	isix_set_irq_vectors_base( &_exceptions_vectors );
    //! Deinitialize RCC
    LL_RCC_DeInit();
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
	LL_FLASH_EnablePrefetch();
	LL_FLASH_EnableDataCache();
	LL_FLASH_EnableInstCache();
	//! Set MCU Prescallers
	LL_RCC_SetAHBPrescaler( LL_RCC_SYSCLK_DIV_1 );
	LL_RCC_SetAPB2Prescaler( LL_RCC_APB2_DIV_2 );
	LL_RCC_SetAPB1Prescaler( LL_RCC_APB1_DIV_4 );
	//! Enable HSE generator
	LL_RCC_HSE_Enable();
	for( int i=0; i<retries; ++i ) {
		if(LL_RCC_HSE_IsReady()) {
			break;
		}
	}
	if( !LL_RCC_HSE_IsReady() ) {
		return false;
	}
    //Enable overdrive mode
	//Enable clocks for GPIOS
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA|LL_AHB1_GRP1_PERIPH_GPIOB|
		LL_AHB1_GRP1_PERIPH_GPIOC|LL_AHB1_GRP1_PERIPH_GPIOD|LL_AHB1_GRP1_PERIPH_GPIOE|
		LL_AHB1_GRP1_PERIPH_GPIOF| LL_AHB1_GRP1_PERIPH_GPIOG|LL_AHB1_GRP1_PERIPH_GPIOH|
		LL_AHB1_GRP1_PERIPH_GPIOI|LL_AHB1_GRP1_PERIPH_GPIOJ|LL_AHB1_GRP1_PERIPH_GPIOK);

	// Setup overdrive mode
	LL_APB1_GRP1_EnableClock(RCC_APB1ENR_PWREN);
	__sync_synchronize();
	LL_PWR_EnableOverDriveMode();
	for( auto r=0; r<retries; ++r ) {
		if(LL_PWR_IsActiveFlag_OD()) {
			break;
		}
	}
	if(!LL_PWR_IsActiveFlag_OD()) {
		return false;
	}
	LL_PWR_EnableOverDriveSwitching();
	for( auto r=0; r<retries; ++r ) {
		if(LL_PWR_IsActiveFlag_ODSW()) {
			break;
		}
	}
	if(!LL_PWR_IsActiveFlag_ODSW()) {
		return false;
	}
#if 1
	//Configure PLL
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_4, 180, LL_RCC_PLLP_DIV_2);
	LL_RCC_PLL_Enable();
	for( auto r=0; r<retries; ++r ) {
		if( LL_RCC_PLL_IsReady() ) {
			break;
		}
	}
	if( !LL_RCC_PLL_IsReady() ) {
		return false;
	}
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
	for( auto r=0; r<retries; ++r ) {
		if( LL_RCC_GetSysClkSource() == LL_RCC_SYS_CLKSOURCE_STATUS_PLL ) {
			break;
		}
	}
	// Config domain clock for LCD_TFT`
    LL_RCC_PLLSAI_ConfigDomain_LTDC(LL_RCC_PLLSOURCE_HSE,LL_RCC_PLLSAIM_DIV_4, 192, LL_RCC_PLLSAIR_DIV_7, LL_RCC_PLLSAIDIVR_DIV_16);
	LL_RCC_PLLSAI_Enable();
	for( auto r=0; r<retries; ++r ) {
		if( LL_RCC_PLLSAI_IsReady() ) {
			break;
		}
	}
	if( !LL_RCC_PLLSAI_IsReady() ) {
		return false;
	}

	return  LL_RCC_GetSysClkSource() == LL_RCC_SYS_CLKSOURCE_STATUS_PLL;
#endif
	return true;
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

	if( uc_periph_setup() && !periph::memory::sdram_setup()) {
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

