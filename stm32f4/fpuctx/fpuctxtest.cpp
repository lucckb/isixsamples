#include <isix.h>
#include <stm32lib.h>
#include <foundation/dbglog.h>
#include <usart_simple.h>
#include "config.hpp"
#include <stm32system.h>
#include <stm32rcc.h>
#include <stm32gpio.h>
#include <stm32tim.h>
#include <stm32adc.h>
#include <stm32dma.h>
#include <stm32pwr.h>
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
void uc_periph_setup()
{
    stm32::rcc_flash_latency( config::HCLK_HZ );
    stm32::rcc_pll1_sysclk_setup( stm32::e_sysclk_hse_pll, config::XTAL_HZ , config::HCLK_HZ );
    stm32::rcc_pclk2_config(  RCC_HCLK_Div2 );
    stm32::rcc_pclk1_config(  RCC_HCLK_Div4 );
    //Setup NVIC vector at begin of flash
    SCB->VTOR = NVIC_VectTab_FLASH;
}

/* ------------------------------------------------------------------ */
extern "C"
{

void fputest_fill_and_add(int start );
int fputest_fill_and_add_check(int start );

/* ------------------------------------------------------------------ */
//! This function is called just before call global constructors
void _external_startup(void)
{

	//Initialize system perhipheral
	uc_periph_setup();

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

	stm32::systick_config( isix::ISIX_HZ * (config::HCLK_HZ/(8*MHZ)) );
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
	//Main function
	virtual void main()
	{
		volatile float ala_z = 1.2;
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
			ala_z += 1.1;
			dbprintf("VAR1=%d", (int)ala_z);
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

class math_task: public isix::task_base
{
public:
	//Constructor
	math_task(int begin )
		: task_base(STACK_SIZE,TASK_PRIO),
        m_begin( begin )
	{
    }
protected:
	//Main function
	virtual void main()
	{
        fputest_fill_and_add(m_begin);
        for(unsigned it=0;;++it)
		{
            const int fret =  fputest_fill_and_add_check(m_begin);
            if( fret ) {
                dbprintf("Assert failed at %i in %i iteration %i", fret, m_begin, it );
                break;
            }
		}
	}
private:
		static const unsigned STACK_SIZE = 512;
		static const unsigned TASK_PRIO = 3;
        int m_begin {};
};


/* ------------------------------------------------------------------ */

}	//namespace app end
/* ------------------------------------------------------------------ */
namespace {
    
    isix::semaphore usem(1, 1);
    void usart_lock() {
        usem.wait(isix::ISIX_TIME_INFINITE);
    }
    void usart_unlock() {
        usem.signal();
    }
}
/* ------------------------------------------------------------------ */
//App main entry point
int main()
{
#ifdef PDEBUG
    stm32::usartsimple_init( USART2,115200,true, config::PCLK1_HZ, config::PCLK2_HZ );
#endif
    dblog_init_putc_locked( stm32::usartsimple_putc, NULL, usart_lock, usart_unlock );
	 dbprintf(" Exception presentation app using ISIXRTOS ");
	//The blinker class
	static app::ledblink led_blinker;
	//The ledkey class
	static app::math_task t1(10);
	static app::math_task t2(20);
    static app::math_task t3(30);
    static app::math_task t4(40);
    static app::math_task t5(50);
    static app::math_task t6(60);
	//Start the isix scheduler
	isix::isix_start_scheduler();
}

/* ------------------------------------------------------------------ */

