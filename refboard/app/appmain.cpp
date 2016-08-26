/*
 * =====================================================================================
 *
 *       Filename:  appmain.cpp
 *
 *    Description:  Application startup 
 *
 *        Version:  1.0
 *        Created:  25.08.2016 21:49:00
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Lucjan Bryndza (LB), lbryndza.p(at)boff(dot)pl
 *   Organization:  BoFF
 *
 * =====================================================================================
 */

#include <config.h>
#include <foundation/dbglog.h>
#include <usart_simple.h>
#include <stm32gpio.h>
#include <isix.h>

static const auto LED_PORT = GPIOG;

#ifdef PDEBUG
namespace {
namespace usart_debug {
	isix::semaphore m_ulock_sem { 1, 1 };
	void lock() 
	{
		m_ulock_sem.wait( ISIX_TIME_INFINITE );
	}
	void unlock() 
	{
		m_ulock_sem.signal();
	}
}}
#endif


void led_blink( void* ) {

	using namespace stm32;
	gpio_config_ext( LED_PORT, 0xf0, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_SPEED_2MHZ );
	 for(int r=0;;r++) {
		dbprintf(" SDIO test %i", r );
		gpio_set_clr_mask( LED_PORT, ~gpio_get_mask(GPIOF,0xf000)>>8, 0xf0 );
		isix::wait_ms( 500 );
	}
}



int main() {
	dblog_init_locked( stm32::usartsimple_putc, nullptr, usart_debug::lock,
			usart_debug::unlock, stm32::usartsimple_init,
			USART1,115200, false, CONFIG_PCLK1_HZ, CONFIG_PCLK2_HZ );
	isix::task_create( led_blink, nullptr, 512, isix::get_min_priority() );
	isix::start_scheduler();
	return 0;
}


