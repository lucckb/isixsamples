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
#include <isixdrv/i2c_bus.hpp>
#include "tftdemo.hpp"
#include "app_env.hpp"
#include <board/si5351.hpp>

static const auto LED_PORT = GPIOG;
namespace app {
	void codec_task( fnd::bus::ibus& bus );
namespace tcp {
	void init();

}}

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


void led_blink( void*  arg ) {

	using namespace stm32;
	gpio_config_ext( LED_PORT, 0xf0, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_SPEED_2MHZ );

	//Test for environment dupa
	int res;
		res = app::setenv( 0, "dupa", 5 );
		dbprintf( "Result for set %i", res );
		isix_wait_ms(100);
	dbprintf( "Setenv results %i\n", res );
	if( !res ) {
		for( int i=0; i<8; ++i ) {
			char buf[8] {};
			res = app::getenv( 0, buf, sizeof buf );
			dbprintf( "Setenv results %i %s\n", res, buf );
		}
	}
	//Test ada fruit board
	auto &si = *reinterpret_cast<Si5351*>( arg );
	si.init( SI5351_CRYSTAL_LOAD_0PF, 0 );
	si.set_freq( 1410000000ULL, 0ULL, SI5351_CLK0 );
	si.set_freq(  710000000ULL, 0ULL, SI5351_CLK1 );
	si.set_freq(  810000000ULL, 0ULL, SI5351_CLK2 );


	// Test the APP environment
	 for(int r=0;;r++) {
		//dbprintf(" SDIO test %i", r );
		gpio_set_clr_mask( LED_PORT, r<<4, 0xf0 );
		isix::wait_ms( 500 );
	}
}



void codec_test_setup()
{
	///Isix thread for I2c bus
	static stm32::drv::i2c_bus m_i2c2 { stm32::drv::i2c_bus::busid::i2c2_alt, 100000 };
	static isix::thread m_thr { isix::thread_create_and_run(
			2048, 3, isix_task_flag_newlib,
			std::bind( &app::codec_task, std::ref(m_i2c2) )
		)
	};
}

int main() {
	dblog_init_locked( stm32::usartsimple_putc, nullptr, usart_debug::lock,
			usart_debug::unlock, stm32::usartsimple_init,
			USART1,115200, false, CONFIG_PCLK1_HZ, CONFIG_PCLK2_HZ );
	//The ledkey class
	static app::tft_tester ft;
	static dev::keypad kp( ft.get_frame() );
	//I2c bus  temporary for tests
	static stm32::drv::i2c_bus m_i2c { stm32::drv::i2c_bus::busid::i2c1_alt , 400000 };
	//FIXME: Temporary disabled due to conflict in the DMA with I2S3
	//app::initenv( m_i2c );
	//static  Si5351 si5351( m_i2c );
	app::tcp::init();
	codec_test_setup();
	//Blink task create
	//isix::task_create( led_blink, &si5351, 2048, isix::get_min_priority() );
	isix::start_scheduler();
	return 0;
}


