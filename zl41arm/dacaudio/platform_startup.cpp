/*
 * platform_setup.cpp
 *  Platform initializaton specific code
 *  Created on: 20 lis 2013
 *      Author: lucck
 */

#include <config/conf.h>
#include <functional>
#include <stm32system.h>
#include <stm32rcc.h>
#include <stm32pwr.h>
#include <stm32adc.h>
#include <isix.h>
#include <stm32crashinfo.h>
#include <stm32syscfg.h>
#include <stm32dma.h>
#include <isix/arch/irq.h>

namespace drv {
namespace board {

namespace {


//SysTimer values
constexpr unsigned MHZ = 1000000;


/** Cortex stm32 System setup
 * Clock and flash configuration for selected rate
 */
unsigned uc_periph_setup()
{
	//Enable or disable used GPIO ports
	stm32::rcc_ahb1_periph_clock_cmd( RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|
			RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOH, true );
	stm32::rcc_ahb1_periph_clock_cmd( RCC_AHB1Periph_DMA1|RCC_AHB1Periph_DMA2, true );
	//Start the stepup converter
	//Setup now other stupp
	stm32::rcc_flash_latency( CONFIG_HCLK_HZ );
	stm32::rcc_hse_config(RCC_HSE_ON);
	stm32::rcc_wait_for_hse_startup();
	constexpr auto M = CONFIG_XTAL_HZ / 1000000;
	stm32::rcc_pll_config( RCC_PLLSource_HSE, M, 240, 2 , 5 );
	stm32::rcc_pll_cmd( true );
	for( auto r=0; r<100000; ++r ) {
		if( !stm32::rcc_get_flag_status( RCC_FLAG_PLLRDY ) )
			break;
	}
    stm32::rcc_pclk2_config(  RCC_HCLK_Div2 );
    stm32::rcc_pclk1_config(  RCC_HCLK_Div4 );
    stm32::rcc_hclk_config( RCC_SYSCLK_Div1 );
	//! Enable external perhipheral supply
    //Enable GPIO system compensation cell block for gpio > 50MHz
    stm32::rcc_ahb2_periph_clock_cmd( RCC_APB2Periph_SYSCFG, true );
    stm32::syscfg_compensation_cell_cmd( true );
    //Configure common ADCS
    stm32::rcc_apb2_periph_clock_cmd( RCC_APB2Periph_ADC, true );
	stm32::adc_common_init( ADC_Mode_Independent, ADC_Prescaler_Div4, ADC_DMAAccessMode_Disabled, ADC_TwoSamplingDelay_5Cycles );
	//Setup DMA
	stm32::rcc_ahb1_periph_clock_cmd( RCC_AHB1Periph_DMA2, true );
    //Setup NVIC vector at begin of flash
    SCB->VTOR = NVIC_VectTab_FLASH;
    stm32::rcc_sysclk_config( RCC_SYSCLKSource_PLLCLK );
    // Enable PLL
    constexpr auto PLL_SRC = 0x08;
    for( auto r=0; r<100000; ++r ) {
    	if( stm32::rcc_get_sysclk_source() != PLL_SRC )
    		break;
    }
    return CONFIG_HCLK_HZ;
}

}	//Unnamed NS

extern "C" {

void _external_exit();

//! This function is called just before call global constructors
void _external_startup(void)
{
#ifdef PDEBUG
	//Give a chance a JTAG to reset the CPU
	for(unsigned i=0; i<1000000; i++) stm32::nop();
#endif
	/* Enable the PWR domain */
	stm32::rcc_apb1_periph_clock_cmd(  RCC_APB1Periph_PWR, true );
	//If it is not software reset exit
#ifndef PDEBUG
	if( stm32::pwr_get_flag_status( PWR_FLAG_SB )
		|| stm32::rcc_get_flag_status( RCC_FLAG_IWDGRST ) )
#else
	if( true )
#endif
	{
		//Clear flags
		stm32::pwr_clear_flag( PWR_FLAG_SB );
		stm32::rcc_clear_flag();
		stm32::pwr_wake_up_pin_cmd( false );

		//Initialize system perhipheral
		const auto volatile freq = uc_periph_setup();


		//1 bit for preemtion priority
		isix_set_irq_priority_group( isix_cortexm_group_pri7 );

		//Initialize isix
		isix::init(freq);

	}
	else
	{
		_external_exit();
	}
}

//!External exit called after global constructors
void _external_exit(void)
{
	//Used gpio port setup
	stm32::rcc_ahb1_periph_clock_cmd( RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|
			RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOH, false );
	stm32::rcc_ahb1_periph_clock_cmd( RCC_AHB1Periph_DMA1|RCC_AHB1Periph_DMA2, false );
	stm32::pwr_wake_up_pin_cmd( true );
	stm32::pwr_enter_standby_mode();
}

#ifdef PDEBUG
//Crash info interrupt handler
void __attribute__((__interrupt__,naked)) hard_fault_exception_vector(void)
{
	cm3_hard_hault_regs_dump();
}
#endif

} /* extern C */


}}	//NS drv

