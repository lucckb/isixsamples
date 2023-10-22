#include <isix.h>
#include <stm32lib.h>
#include <foundation/sys/dbglog.h>
#include <usart_simple.h>
#include "config.hpp"
#include <stm32system.h>
#include <stm32rcc.h>
#include <stm32gpio.h>
#include <stm32tim.h>
#include <stm32adc.h>
#include <stm32dma.h>
#include <stm32pwr.h>
#include <isix/arch/irq.h>
 
namespace {
 

//Number of isix threads
const unsigned ISIX_NUM_PRIORITIES = 4;
//SysTimer values
const unsigned MHZ = 1000000;

 
/** Cortex stm32 System setup
 * Clock and flash configuration for selected rate
 */
void uc_periph_setup()
{
    stm32::rcc_flash_latency( config::HCLK_HZ );
    stm32::rcc_pll1_sysclk_setup( stm32::e_sysclk_hse_pll, config::XTAL_HZ , config::HCLK_HZ );
    stm32::rcc_pclk2_config(  RCC_HCLK_Div2 );
    stm32::rcc_pclk1_config(  RCC_HCLK_Div4 );
    //Setup NVIC vector at begin of flash
#if CONFIG_ISIX_SBL_BOOTLOADER_REMAP
    SCB->VTOR = 0x08004000;
#else
    SCB->VTOR = NVIC_VectTab_FLASH;
#endif
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
	isix::init(config::HCLK_HZ);

}
 
} /* extern C */

 
}
 
namespace app
{

struct ledblink
{
	ledblink() :  LED_PORT(GPIOE)
	{
		using namespace stm32;
		gpio_clock_enable( LED_PORT, true);
		gpio_abstract_config(LED_PORT, LED_PIN, AGPIO_MODE_OUTPUT_PP, AGPIO_SPEED_HALF );
	}
	void main() noexcept
	{
		volatile float ala_z = 1.2;
		while(true)
		{
			//Enable LED
			stm32::gpio_clr( LED_PORT, LED_PIN );
			//Wait time
			isix::wait_ms( BLINK_TIME );
			//Disable LED
			stm32::gpio_set( LED_PORT, LED_PIN );
			//Wait time
			isix::wait_ms( BLINK_TIME );
			ala_z *= 1.1;
			dbprintf("VAR1=%d", (int)ala_z);
		}
	}
	GPIO_TypeDef * const LED_PORT;
	static const unsigned LED_PIN = 14;
	static const unsigned BLINK_TIME = 500;
};
 

struct ledkey
{
	void main() noexcept
	{
		volatile float ala_j = 1.2;
		for(;;)
		{
			isix::wait_ms( 600 );
			ala_j += 12.0;
			dbprintf("VAR2=%d", (int)ala_j);
		}
	}
};

 

}	//namespace app end
 
//App main entry point
int main()
{
	dblog_init( stm32::usartsimple_putc, NULL, stm32::usartsimple_init,
		USART2,115200,true, config::PCLK1_HZ, config::PCLK2_HZ );
	dbprintf(" Exception presentation app using ISIXRTOS ");

	static app::ledblink led_blinker;
	static isix::thread blinker_thr = isix::thread_create_and_run(
		2048, 3, isix_task_flag_newlib, &app::ledblink::main, &led_blinker);

	static app::ledkey led_key;
	static isix::thread key_thr = isix::thread_create_and_run(
		2048, 3, isix_task_flag_newlib, &app::ledkey::main, &led_key);

	isix::start_scheduler();
}

int _write (int /*file*/, const void * /*ptr*/, size_t /*len*/)  { return -1; }
int _read (int /*file*/, void * /*ptr*/, size_t /*len*/)  { return -1; }
off_t _lseek (int /*file*/, off_t /*ptr*/, int /*dir*/)  { return -1; }
int _close (int /*file*/)  { return -1; }
