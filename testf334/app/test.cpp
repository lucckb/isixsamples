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
#include <foundation/sys/dbglog.h>
#include <isix.h>
#include <periph/dt/dts.hpp>
#include <periph/drivers/serial/uart_early.hpp>
#include <periph/dma/dma.hpp>
#include "hrtim_test.hpp"


#if 0
#include <foundation/drv/lcd/ssd1306.hpp>
#include <foundation/drv/lcd/uc1601_display.hpp>
#include <isixdrv/spi_master.hpp>
#include <isixdrv/i2c_bus.hpp>
#include <isixdrv/gpioout.hpp>
#include "resources.hpp"
#endif
#include <periph/gpio/gpio.hpp>
#include <periph/bus/transfer.hpp>

//SSD1306 display driver
//https://github.com/kmm/SS1306.git


namespace app {
//! Test thread for new display library
void test_thread(void*)
{
#if 0
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
#endif
	//err = disp.progress_bar( 32,16,80,16,50,100);
	//err = disp.draw_hline( 0, 63, 80, fnd::drv::lcd::color::black);
	err = disp.show_icon( 12,16,&app::res::manual_icon );
	dbprintf("draw hline status %i", err );
#endif
	app::hrtim_test_init();
	for(;;) {
		isix::wait_ms(1000);
		dbprintf("Dupa >>>");
	}
}

}


int main()
{
	isix::wait_ms( 500 );
	static isix::semaphore m_ulock_sem { 1, 1 };
	dblog_init_locked(
		[](int ch, void*) {
			return periph::drivers::uart_early::putc(ch);
		},
		nullptr,
		[]() {
			m_ulock_sem.wait(ISIX_TIME_INFINITE);
		},
		[]() {
			m_ulock_sem.signal();
		},
		periph::drivers::uart_early::init,
		"serial0", 115200
	);
	isix::task_create( app::test_thread, nullptr, 512, isix::get_min_priority() );
		dbprintf("<<<< You welcome >>>>");
	isix::start_scheduler();
	return 0;
}
