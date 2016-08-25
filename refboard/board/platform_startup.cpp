
#include "config.h"
#include <functional>
#include <stm32system.h>
#include <stm32rcc.h>
#include <stm32pwr.h>
#include <stm32adc.h>
#include <stm32exti.h>
#include <stm32rtc.h>
#include <isix.h>
#include <stm32crashinfo.h>
#include <stm32syscfg.h>
#include <stm32dma.h>

namespace drv {
namespace board {
namespace {


//Number of isix threads
constexpr unsigned ISIX_NUM_PRIORITIES = 4;
//SysTimer values
constexpr unsigned MHZ = 1000000;




/** Cortex stm32 System setup
 * Clock and flash configuration for selected rate
 */
unsigned uc_periph_setup()
{
	using namespace stm32;

    rcc_flash_latency(CONFIG_HCLK_HZ);
    auto freq = rcc_pll1_sysclk_setup( stm32::e_sysclk_hse_pll, CONFIG_XTAL_HZ , CONFIG_HCLK_HZ );
    rcc_pclk2_config(  RCC_HCLK_Div2 );
    rcc_pclk1_config(  RCC_HCLK_Div4 );
    //Setup NVIC vector at begin of flash
    //Setup NVIC vector at begin of flash
#if CONFIG_WITH_SBL_BOOTLOADER_ENABLED
    SCB->VTOR = NVIC_VectTab_FLASH + 0x4000;
#else
    SCB->VTOR = NVIC_VectTab_FLASH;
#endif

	return freq;
}

}
}
}


extern "C" {

//! This function is called just before call global constructors
void _external_startup(void)
{
	using namespace stm32;

	//Give a chance a JTAG to reset the CPU
	for(unsigned i=0; i<1000000; i++) stm32::nop();

	//Initialize system perhipheral
	const auto freq = drv::board::uc_periph_setup();

	//1 bit for preemtion priority
	nvic_priority_group(NVIC_PriorityGroup_1);

	//System priorities
	nvic_set_priority(PendSV_IRQn,1,0x7);

	//System priorities
	nvic_set_priority(SVCall_IRQn,1,0x7);

	//Set timer priority
	nvic_set_priority(SysTick_IRQn,1,0x7);

	//Initialize isix
	isix::init(ISIX_CONFIG_NUM_PRIORITIES);

	//Configure systic timer
	systick_config( ISIX_HZ * (freq/(8000000)) );
}



//Crash info interrupt handler
void __attribute__((__interrupt__,naked)) hard_fault_exception_vector(void)
{

}

} /* extern C */


