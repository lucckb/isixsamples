#include <config/conf.h>
#include <functional>
#include <stm32_ll_rcc.h>
#include <stm32_ll_pwr.h>
#include <stm32_ll_adc.h>
#include <stm32_ll_gpio.h>
#include <isix.h>
#include <stm32_ll_system.h>
#include <stm32_ll_dma.h>
#include <isix/arch/irq.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_rcc.h>
#include <boot/arch/arm/cortexm/irq_vectors_table.h>
#include <boot/arch/arm/cortexm/crashinfo.h>


namespace drv {
namespace board {
namespace {


//Number of isix threads
constexpr unsigned ISIX_NUM_PRIORITIES = 4;
//SysTimer values
constexpr unsigned MHZ = 1000000;

const auto MCO1_PORT =  GPIOA;
constexpr auto MCO1_PIN = 8;


/** Cortex stm32 System setup
 * Clock and flash configuration for selected rate
 */
unsigned system_config()
{
	constexpr auto retries=100000;
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_7);
	//! Enable HSE generator
	LL_RCC_HSE_Enable();
	for( int i=0; i<retries; ++i ) {
		if(LL_RCC_HSE_IsReady()) {
			break;
		}
	}
	if( !LL_RCC_HSE_IsReady() ) {
		for(;;) {};
	}
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_25, 336, LL_RCC_PLLP_DIV_2);
	LL_RCC_PLL_ConfigDomain_48M(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_25, 336, LL_RCC_PLLQ_DIV_7);
	LL_RCC_PLL_Enable();
	for( auto r=0; r<retries; ++r ) {
		if( LL_RCC_PLL_IsReady() ) {
			break;
		}
	}
	if( !LL_RCC_PLL_IsReady() ) {
		for(;;) {}
	}
	LL_RCC_SetAHBPrescaler( LL_RCC_SYSCLK_DIV_1 );
	LL_RCC_SetAPB2Prescaler( LL_RCC_APB2_DIV_2 );
	LL_RCC_SetAPB1Prescaler( LL_RCC_APB1_DIV_4 );

	//Enable GPIO system compensation cell block for gpio > 50MHz
	LL_AHB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
	LL_SYSCFG_EnableCompensationCell();

	isix_set_irq_vectors_base( &_exceptions_vectors );

    // Enable main PLL
 	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
	for( auto r=0; r<retries; ++r ) {
		if( LL_RCC_GetSysClkSource() == LL_RCC_SYS_CLKSOURCE_STATUS_PLL ) {
			break;
		}
	}
    return CONFIG_HCLK_HZ;
}


void periph_config()
{
	//Configure common ADCS
	LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_ADC);
	LL_AHB1_GRP1_EnableClock( LL_AHB1_GRP1_PERIPH_DMA1|LL_AHB1_GRP1_PERIPH_DMA2);
	LL_AHB1_GRP1_EnableClock( LL_AHB1_GRP1_PERIPH_GPIOA| LL_AHB1_GRP1_PERIPH_GPIOB|
		LL_AHB1_GRP1_PERIPH_GPIOC|LL_AHB1_GRP1_PERIPH_GPIOD|
		LL_AHB1_GRP1_PERIPH_GPIOE|LL_AHB1_GRP1_PERIPH_GPIOF|
		LL_AHB1_GRP1_PERIPH_GPIOG|LL_AHB1_GRP1_PERIPH_GPIOH
	);

	//Configure MCO1 as master clock output for external devs
	LL_GPIO_InitTypeDef port_cnf {
		.Pin = 1U<<MCO1_PIN,
		.Mode = LL_GPIO_MODE_ALTERNATE,
		.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH,
		.OutputType = LL_GPIO_OUTPUT_PUSHPULL,
		.Pull = LL_GPIO_PULL_NO,
		.Alternate = LL_GPIO_AF_0
	};
	LL_GPIO_Init(MCO1_PORT, &port_cnf);
	LL_RCC_ConfigMCO(LL_RCC_MCO1SOURCE_HSE, LL_RCC_MCO1SOURCE_HSE);
}


//NS end

}
}
}


extern "C" {

//! This function is called just before call global constructors
void _external_startup(void)
{

	//Give a chance a JTAG to reset the CPU
	for(unsigned i=0; i<1000000; i++)
		asm volatile("nop\n");

	//Initialize system perhipheral
	const auto freq = drv::board::system_config();

	//Configure perhipherals
	drv::board::periph_config();

	//1 bit for preemtion priority
	isix_set_irq_priority_group( isix_cortexm_group_pri7 );

	//Initialize isix
	isix::init(freq);

}



//Crash info interrupt handler
[[gnu::naked]]
void hard_fault_exception_vector(void)
{
	cm3_hard_hault_regs_dump();
}

#ifdef PDEBUG
int _write (int /*file*/, const void * /*ptr*/, size_t /*len*/)  { return -1; }
int _read (int /*file*/, void * /*ptr*/, size_t /*len*/)  { return -1; }
off_t _lseek (int /*file*/, off_t /*ptr*/, int /*dir*/)  { return -1; }
int _close (int /*file*/)  { return -1; }
#endif // PDEBUG

}

