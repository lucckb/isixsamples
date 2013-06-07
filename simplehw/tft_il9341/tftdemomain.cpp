#include <isix.h>
#include <stm32lib.h>
#include <dbglog.h>
#include <usart_simple.h>
#include "config.h"
#include <stm32system.h>
#include <stm32rcc.h>
#include <stm32gpio.h>
#include <cctype>
#include <cstring>
#include <stm32gpio.h>
/* ------------------------------------------------------------------ */
namespace {
/* ------------------------------------------------------------------ */

//Number of isix threads
const unsigned ISIX_NUM_PRIORITIES = 4;
//SysTimer values
const unsigned MHZ = 1000000;

/* ------------------------------------------------------------------ */
/** Cortex stm32 System setup
 * Clock and flash configuration for selected rate
 */
uint32_t uc_periph_setup()
{
    stm32::rcc_flash_latency( CONFIG_HCLK_HZ );
    const uint32_t freq = stm32::rcc_pll1_sysclk_setup( stm32::e_sysclk_hse_pll, CONFIG_XTAL_HZ , CONFIG_HCLK_HZ );
    stm32::rcc_pclk2_config(  RCC_HCLK_Div1 );
    stm32::rcc_pclk1_config(  RCC_HCLK_Div2 );
    //Setup NVIC vector at begin of flash
    SCB->VTOR = NVIC_VectTab_FLASH;
    return freq;
}
/* ------------------------------------------------------------------ */
extern "C"
{

/* ------------------------------------------------------------------ */
//! This function is called just before call global constructors
void _external_startup(void)
{

	//Initialize system perhipheral
	const uint32_t freq = uc_periph_setup();

	//1 bit for preemtion priority
	stm32::nvic_priority_group(NVIC_PriorityGroup_1);

	//System priorities
	stm32::nvic_set_priority(PendSV_IRQn,1,0x7);

	//System priorities
	stm32::nvic_set_priority(SVCall_IRQn,1,0x7);

	//Set timer priority
	stm32::nvic_set_priority(SysTick_IRQn,1,0x7);

	//Initialize isix
	isix::isix_init(ISIX_NUM_PRIORITIES);

	stm32::systick_config( isix::ISIX_HZ * (freq/(8*MHZ)) );
}
/* ------------------------------------------------------------------ */
} /* extern C */

/* ------------------------------------------------------------------ */
}
/* ------------------------------------------------------------------ */
namespace app
{

/* ------------------------------------------------------------------ */
class ledblink: public isix::task_base
{
public:
	//Constructor
	ledblink() : task_base(STACK_SIZE,TASK_PRIO), LED_PORT(GPIOE)
	{
		using namespace stm32;
		gpio_clock_enable( LED_PORT, true);
		gpio_abstract_config(LED_PORT, LED_PIN, AGPIO_MODE_OUTPUT_PP, AGPIO_SPEED_HALF );
	}
protected:
	//Main functionQMonikQ
	virtual void main()
	{
		while(true)
		{
			//Enable LED
			stm32::gpio_clr( LED_PORT, LED_PIN );
			//Wait time
			isix::isix_wait( isix::isix_ms2tick(BLINK_TIME) );
			//Disable LED
			stm32::gpio_set( LED_PORT, LED_PIN );
			//Wait time
			isix::isix_wait( isix::isix_ms2tick(BLINK_TIME) );
		}
	}
private:
	static const unsigned STACK_SIZE = 2048;
	static const unsigned TASK_PRIO = 3;
	GPIO_TypeDef * const LED_PORT;
	static const unsigned LED_PIN = 14;
	static const unsigned BLINK_TIME = 500;
};

/* ------------------------------------------------------------------ */

class tft_tester: public isix::task_base
{
public:
	//Constructor
	tft_tester()
		: task_base(STACK_SIZE,TASK_PRIO)
	{
	}
protected:
	//Main function
	virtual void main()
	{

			isix::isix_wait_ms( 1000 );
	}
private:
		static const unsigned STACK_SIZE = 2048;
		static const unsigned TASK_PRIO = 3;
};



/* ------------------------------------------------------------------ */

}	//namespace app end
/* ------------------------------------------------------------------ */
//App main entry point
int main()
{
	 dblog_init( stm32::usartsimple_putc, NULL, stm32::usartsimple_init,
	    		USART2,115200,true, CONFIG_PCLK1_HZ, CONFIG_PCLK2_HZ );
	 dbprintf("TFT Tester. Good Morning");
	//The blinker class
	static app::ledblink led_blinker;
	//The ledkey class
	static app::tft_tester ft;
	//static app::mmc_host_tester ht;
	isix::isix_wait_ms(1000);
	//Start the isix scheduler
	isix::isix_start_scheduler();
}

/* ------------------------------------------------------------------ */

