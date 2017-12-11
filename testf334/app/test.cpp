/*
 * =====================================================================================
 *
 *       Filename:  test.cpp
 *
 *    Description:  Test application
 *
 *        Version:  1.0
 *        Created:  09.12.2017 22:40:19
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Lucjan Bryndza (LB), lbryndza.p@boff.pl
 *   Organization:  BoFF
 *
 * =====================================================================================
 */
#include <config/conf.h>
#include <foundation/dbglog.h>
#include <usart_simple.h>
#include <isix.h>


//SSD1306 display driver
https://github.com/kmm/SS1306.git


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


void test_thread(void*)
{
	for(int cnt=0;;++cnt) {
		dbprintf("Hello from thread %i",cnt);
		isix::wait_ms(1000);
	}
}

//extern "C" void std::abort() {}

//extern "C" void _sbrk() {}


int main() {
	isix::wait_ms( 500 );
	dblog_init_locked( stm32::usartsimple_putc, nullptr, usart_debug::lock,
			usart_debug::unlock, stm32::usartsimple_init,
			USART1,115200, USARTSIMPLE_FL_ALTERNATE_PC, CONFIG_PCLK1_HZ, CONFIG_PCLK2_HZ );
	isix::task_create( test_thread, nullptr, 512, isix::get_min_priority() );
		dbprintf("<<<< You welcome >>>>");
	isix::start_scheduler();
	return 0;
}
