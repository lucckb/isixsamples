
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

constexpr auto PLL_M = 25;
constexpr auto PLL_N = 336;
constexpr auto PLL_P = 2;	//168MHZ master clock
constexpr auto PLL_Q = 7;	//48MHz for USB clk


const auto MCO1_PORT =  GPIOA;
constexpr auto MCO1_PIN = 8;


/** Cortex stm32 System setup
 * Clock and flash configuration for selected rate
 */
unsigned system_config()
{
	using namespace stm32;

    rcc_flash_latency(CONFIG_HCLK_HZ);
	rcc_hse_config( RCC_HSE_ON );
	rcc_wait_for_hse_startup();
	rcc_pll_config( RCC_PLLSource_HSE, PLL_M, PLL_N, PLL_P, PLL_Q );
	rcc_pll_cmd( true );
	for( auto r=0; r<100000; ++r ) {
		if( !rcc_get_flag_status( RCC_FLAG_PLLRDY ) )
			break;
	}

    rcc_pclk2_config( RCC_HCLK_Div2 );
    rcc_pclk1_config( RCC_HCLK_Div4 );
	rcc_hclk_config(  RCC_SYSCLK_Div1 );
	//Enable GPIO system compensation cell block for gpio > 50MHz
    rcc_ahb2_periph_clock_cmd( RCC_APB2Periph_SYSCFG, true );
    syscfg_compensation_cell_cmd( true );



    //Setup NVIC vector at begin of flash
    //Setup NVIC vector at begin of flash
#if CONFIG_WITH_SBL_BOOTLOADER_ENABLED
    SCB->VTOR = NVIC_VectTab_FLASH + 0x4000;
#else
    SCB->VTOR = NVIC_VectTab_FLASH;
#endif

    // Enable main PLL
	rcc_sysclk_config( RCC_SYSCLKSource_PLLCLK );
    constexpr auto PLL_SRC = 0x08;
    for( auto r=0; r<100000; ++r ) {
	if( rcc_get_sysclk_source() != PLL_SRC )
		break;
    }
    return CONFIG_HCLK_HZ;
}


void periph_config()
{
	using namespace stm32;
	//Configure common ADCS
    rcc_apb2_periph_clock_cmd( RCC_APB2Periph_ADC, true );
	adc_common_init( ADC_Mode_Independent, ADC_Prescaler_Div4,
			ADC_DMAAccessMode_Disabled, ADC_TwoSamplingDelay_5Cycles );
	//Setup DMA
	rcc_ahb1_periph_clock_cmd( RCC_AHB1Periph_DMA1|RCC_AHB1Periph_DMA2, true );
	rcc_ahb1_periph_clock_cmd( RCC_AHB1Periph_DMA1|RCC_AHB1Periph_DMA2, true );

	//Configure used GPIOS
	rcc_ahb1_periph_clock_cmd( RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|
		RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOE|
		RCC_AHB1Periph_GPIOF|RCC_AHB1Periph_GPIOG|RCC_AHB1Periph_GPIOH, true );

	//Configure MCO1 as master clock output for external devs
	gpio_config( MCO1_PORT, MCO1_PIN, GPIO_MODE_ALTERNATE, GPIO_PUPD_PULLUP, GPIO_SPEED_100MHZ );
	rcc_mco1_config( RCC_MCO1Source_HSE, RCC_MCO1Div_1 );

}


//NS end

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
	const auto freq = drv::board::system_config();

	//Configure perhipherals
	drv::board::periph_config();

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
	cm3_hard_hault_regs_dump();
}

} /* extern C */


