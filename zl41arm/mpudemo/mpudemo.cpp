#include <isix.h>
#include <stm32lib.h>
#include <foundation/dbglog.h>
#include <foundation/tiny_printf.h>
#include <usart_simple.h>
#include "config.hpp"
#include <stm32system.h>
#include <stm32rcc.h>
#include <stm32gpio.h>
#include <stm32tim.h>
#include <stm32adc.h>
#include <stm32dma.h>
#include <stm32pwr.h>
#include <stm32crashinfo.h>
#include <isix/arch/irq.h>
 
namespace {
 
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
	//Crash info interrupt handler
	void __attribute__((__interrupt__,naked)) hard_fault_exception_vector(void)
	{
		cm3_hard_hault_regs_dump();
	}
	//Isix panic callback
	void isix_kernel_panic_callback( const char* file, int line, const char *msg )
	{
		fnd::tiny_printf("ISIX_PANIC %s:%i %s\r\n", file, line, msg );
	}

 
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

 
class ledblink: public isix::task_base
{
public:
	//Constructor
	ledblink() :  LED_PORT(GPIOE)
	{
		using namespace stm32;
		gpio_clock_enable( LED_PORT, true);
		gpio_abstract_config(LED_PORT, LED_PIN, AGPIO_MODE_OUTPUT_PP, AGPIO_SPEED_HALF );
		start_thread( STACK_SIZE, TASK_PRIO);
	}
protected:
	//Main function
	virtual void main()
	{
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
		}
	}
private:
	static const unsigned STACK_SIZE = 2048;
	static const unsigned TASK_PRIO = 3;
	GPIO_TypeDef * const LED_PORT;
	static const unsigned LED_PIN = 14;
	static const unsigned BLINK_TIME = 500;
};
 

class mpu_demo: public isix::task_base
{
	static inline auto key_port() {
		return GPIOC;
	}
public:
	//Constructor
	mpu_demo()
	{
		using namespace stm32;
		start_thread( STACK_SIZE, TASK_PRIO );
		gpio_clock_enable( key_port(), true );
		gpio_abstract_config(key_port(), NULL_PIN, AGPIO_MODE_INPUT_PULLUP, AGPIO_SPEED_HALF );
		gpio_abstract_config(key_port(), EXEC_PIN, AGPIO_MODE_INPUT_PULLUP, AGPIO_SPEED_HALF );
		gpio_abstract_config(key_port(), STK_PIN, AGPIO_MODE_INPUT_PULLUP, AGPIO_SPEED_HALF );
	}
protected:
	//Main function
	virtual void main()
	{
		for(;;)
		{
			bool knull = stm32::gpio_get( key_port(), NULL_PIN );
			bool kexec = stm32::gpio_get( key_port(), EXEC_PIN );
			bool kstk = stm32::gpio_get( key_port(),  STK_PIN );
			if( !knull && p_null ) {
				dbprintf( "Trying to read from null pointer" );
				mpu_demo* nptr = nullptr;
				dbprintf( "Test %i", nptr->p_null );
			}
			if( !kexec && p_exec ) {
				dbprintf( "Trying to exec code from ram" );
				void(*pfn)() = reinterpret_cast<void(*)()>(0x20000000);
				pfn();
			}
			if( !kstk && p_stk ) {
				dbprintf( "Trying to overflow stack" );
				int lvar;
				volatile auto pvar = &lvar;
				for(int i=0;i<100000;++i ) {
					*pvar-- = 0;
				}
			}
			p_null = knull; p_exec = kexec; p_stk = kstk;
			isix::wait_ms( 25 );
		}
	}
	private:
	private:
		static const unsigned STACK_SIZE = 2048;
		static const unsigned TASK_PRIO = 3;
		static constexpr auto NULL_PIN = 12;
		static constexpr auto EXEC_PIN = 13;
		static constexpr auto STK_PIN = 14;
		bool p_null { true };
		bool p_exec { true };
		bool p_stk { true };
};

 

}	//namespace app end
 
//App main entry point
int main()
{
	 dblog_init( stm32::usartsimple_putc, NULL, stm32::usartsimple_init,
	    		USART2,115200,true, config::PCLK1_HZ, config::PCLK2_HZ );
	 dbprintf("MPU demo");
	//The blinker class
	static app::ledblink led_blinker;
	//The ledkey class
	static app::mpu_demo mpu_demo;
	//Start the isix scheduler
	isix::start_scheduler();
}

 

