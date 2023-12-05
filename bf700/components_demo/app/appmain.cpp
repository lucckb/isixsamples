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

#include <config/conf.h>
#include <isix.h>
#include <foundation/sys/dbglog.h>
#include <periph/drivers/serial/uart_early.hpp>
#include <stm32_ll_gpio.h>
#include "tftdemo.hpp"
#include "app_env.hpp"
#include <board/si5351.hpp>
#include <periph/drivers/i2c/i2c_master.hpp>

static const auto LED_PORT = GPIOG;
namespace app {
	void codec_task( periph::drivers::i2c_master& bus );
	void stdio_test_task(  );
namespace tcp {
	void init();

}}

namespace {

/* Initialize the debug USART */
	auto usart_protected_init() -> void {
		static isix::mutex m_mtx;
		dblog_init_locked(
				[](int ch, void*) {
					return periph::drivers::uart_early::putc(ch);
				},
				nullptr,
				[]() { m_mtx.lock();  },
				[]() { m_mtx.unlock(); },
				periph::drivers::uart_early::open, "serial0", 115200
		);
	}

}


void i2c1bus_test( void*  arg )
{

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


}

static inline void gpio_set_clr_mask(GPIO_TypeDef* port , uint16_t enflags, uint16_t mask)
{
	port->BSRR = (uint32_t)(enflags & mask) | ((uint32_t)( ~enflags & mask)<<16);
}

static inline uint16_t gpio_get_mask(GPIO_TypeDef* port , uint16_t bitmask)
{
	return port->IDR & bitmask;
}

static inline bool gpio_get(GPIO_TypeDef* port , unsigned bit)
{
	return (port->IDR >> (bit))&1;
}

void pulse_test(void*) 
{
	LL_GPIO_InitTypeDef io_cfg {
		.Pin = 0xf0,
		.Mode = LL_GPIO_MODE_OUTPUT,
		.Speed = LL_GPIO_SPEED_FREQ_MEDIUM,
		.OutputType = LL_GPIO_OUTPUT_PUSHPULL,
		.Pull = LL_GPIO_PULL_NO,
		.Alternate = 0
	};
	LL_GPIO_Init( LED_PORT, &io_cfg);
	// Test the APP environment
	 for(int r=0;;r++) {
		gpio_set_clr_mask( LED_PORT, r, 0x20 );
		gpio_set_clr_mask( LED_PORT, ~gpio_get_mask(GPIOB,0xC0), 0xc0 );
		gpio_set_clr_mask( LED_PORT, gpio_get(GPIOE,0)?1<<4:0, 0x10 );
		isix::wait_ms( 10 );
	}
}


void codec_test_setup()
{
	///Isix thread for I2c bus
	static periph::drivers::i2c_master m_i2c2("i2c2");
	static isix::thread m_thr { isix::thread_create_and_run(
			2048, 3, isix_task_flag_newlib,
			std::bind( &app::codec_task, std::ref(m_i2c2) )
		)
	};
}

int main()
{
	isix::wait_ms( 500 );
	usart_protected_init();
	//The ledkey class
	static app::tft_tester ft;
	static isix::thread tester_thr = isix::thread_create_and_run(
		2048, 3, isix_task_flag_newlib, &app::tft_tester::main, &ft);
	static dev::keypad kp( ft.get_frame() );
	//I2c bus  temporary for tests
	//static stm32::drv::i2c_bus m_i2c { stm32::drv::i2c_bus::busid::i2c1_alt , 400000 };
	//FIXME: Temporary disabled due to conflict in the DMA with I2S3
	//app::initenv( m_i2c );
	//static  Si5351 si5351( m_i2c );
	app::tcp::init();
	codec_test_setup();
	//Blink task create
	//isix::task_create( i2c1bus_test, &si5351, 2048, isix::get_min_priority() );
	isix::task_create( pulse_test, nullptr, 512, isix::get_min_priority() );
	//Enable 5V USB
	{
		LL_GPIO_InitTypeDef io_cfg {
		.Pin = LL_GPIO_PIN_3,
		.Mode = LL_GPIO_MODE_OUTPUT,
		.Speed = LL_GPIO_SPEED_FREQ_MEDIUM,
		.OutputType = LL_GPIO_OUTPUT_PUSHPULL,
		.Pull = LL_GPIO_PULL_NO,
		.Alternate = 0
		};
		LL_GPIO_Init( GPIOE, &io_cfg);
		LL_GPIO_SetOutputPin(GPIOE, LL_GPIO_PIN_3);
	}
	isix::start_scheduler();
	return 0;
}


