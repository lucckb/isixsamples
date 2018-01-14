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
#if 0
#include <config/conf.h>
#include <foundation/sys/dbglog.h>
#include <usart_simple.h>
#include <isix.h>

#include <foundation/drv/lcd/ssd1306.hpp>
#include <foundation/drv/lcd/uc1601_display.hpp>
#include <isixdrv/spi_master.hpp>
#include <isixdrv/i2c_bus.hpp>
#include <isixdrv/gpioout.hpp>
#include "resources.hpp"
#include "hrtim_test.hpp"
#endif


//SSD1306 display driver
//https://github.com/kmm/SS1306.git

#if 0

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

//! Test thread for new display library
void test_thread(void*) {
	using smod = drv::spi_device;
	static constexpr stm32::drv::spi_gpio_config spicnf {
		CONFIG_PCLK1_HZ,
		CONFIG_PCLK2_HZ,
		stm32::GPIO_AF_5,
		{ stm32::gpio::pin_desc::PB, 4 },	//MISO
		{ stm32::gpio::pin_desc::PB, 5 },	//MOSI
		{ stm32::gpio::pin_desc::PB, 3 },	//SCK
		{{				//CS
			{ stm32::gpio::pin_desc::PB, 7 },	// MEM_CS - CS0
			{ stm32::gpio::pin_desc::PB, 6 }	// DI_CS -  CS1
		}}
	};
	stm32::drv::spi_master spidev( SPI1, spicnf );
	spidev.enable( true );
	static constexpr int bus_speed = 10E6;
	spidev.set_mode( smod::data_8b|smod::msb_first|smod::polar_cpol_low|smod::phase_1edge, bus_speed/1000, smod::CS1 );
	stm32::drv::gpio_out rst { GPIOB, 8 };
	stm32::drv::gpio_out di { GPIOB, 9 };
	fnd::drv::lcd::ssd1306 disp( spidev, di, rst, smod::CS1, 128, 64 );
	int err = disp.enable(true);
	disp.set_font( &app::res::font_default);
	dbprintf("Disp en status %i",err);
	err = disp.clear();
	dbprintf("Disp clear status %i",err);
#if 1
	err = disp.puts("Ada to nie wypada");
	dbprintf("Disp puts status %i", err );
	err = disp.endl();
	dbprintf("Disp puts status %i", err );
	err = disp.puts("I ma cyce jak szpada");
	dbprintf("Disp puts status %i", err );
	disp.set_font( &app::res::font_big );
	err = disp.setpos(0,32);
	dbprintf("Disp puts setpos %i", err );
	err = disp.puts("1234");
	dbprintf("Disp puts status %i", err );
	app::hrtim_test_init();
#endif
	//err = disp.progress_bar( 32,16,80,16,50,100);
	//err = disp.draw_hline( 0, 63, 80, fnd::drv::lcd::color::black);
	err = disp.show_icon( 12,16,&app::res::manual_icon );
	dbprintf("draw hline status %i", err );
	for(;;) {
		isix::wait_ms(1000);
	}
}



#endif
int main() {
#if 0
	isix::wait_ms( 500 );
	dblog_init_locked( stm32::usartsimple_putc, nullptr, usart_debug::lock,
			usart_debug::unlock, stm32::usartsimple_init,
			USART1,115200, USARTSIMPLE_FL_ALTERNATE_PC, CONFIG_PCLK1_HZ, CONFIG_PCLK2_HZ );
	isix::task_create( test_thread, nullptr, 512, isix::get_min_priority() );
		dbprintf("<<<< You welcome >>>>");
	isix::start_scheduler();
#endif
	return 0;
}
