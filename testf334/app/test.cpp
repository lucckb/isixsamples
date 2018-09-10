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
#include <periph/drivers/spi/spi_master.hpp>
#include <periph/dma/dma.hpp>
#include "hrtim_test.hpp"
#include <periph/drivers/display/mono/ssd1306.hpp>
#include <periph/gpio/gpio.hpp>
#include <periph/blk/transfer.hpp>
#include <periph/core/device.hpp>
#include <periph/core/peripheral_manager.hpp>
#include <periph/dma/dma.hpp>
#include "resources.hpp"
//SSD1306 display driver
//https://github.com/kmm/SS1306.git
// p/x *(SPI_TypeDef*)(0x40000000U+0x00010000U+ 0x00003000U)



namespace app {

/** Test SPI internal driver based on the new periphlib library
 */
void spi_rwtest( periph::drivers::spi_master& spi )  {
	constexpr uint8_t inbuf[6] { 0x90 };
	uint8_t outbuf[6] {};
	static_assert( sizeof outbuf == sizeof inbuf );
	periph::blk::trx_transfer tran(inbuf,outbuf,sizeof outbuf);
	dbprintf("Do spirwtest");
	int err = spi.transaction(1, tran);
	dbprintf("Transfer error status %i", err );
	for( auto a: outbuf ) {
		dbprintf("Return %02x", a);
	}
}

//! Test thread for new display library
void test_thread(void*)
{
	namespace opt = periph::option;
	dbprintf("Before SPI construct");
	periph::drivers::spi_master m_spi("spi1");
	int ret = m_spi.open(ISIX_TIME_INFINITE);
	dbprintf("SPI open status %i", ret);
	ret = m_spi.set_option( opt::speed(10E6) );
	dbprintf("Set option speed %i", ret);
	ret = m_spi.set_option( opt::polarity(opt::polarity::low));
	dbprintf("Set option polarity %i", ret);
    ret = m_spi.set_option( opt::phase(opt::phase::_1_edge));
	dbprintf("Set option phase %i", ret);
    ret = m_spi.set_option( opt::dwidth(8));
	dbprintf("Set option data width %i", ret);
    ret = m_spi.set_option( opt::bitorder(opt::bitorder::msb));
	dbprintf("Set option bitorder %i", ret);
#if 1
	dbprintf("Before SSD");
	periph::display::ssd1306 disp("display0", m_spi);
	dbprintf("After SSD");
	int err = disp.enable(true);
	for(int a=0;a<5;a++) {
	disp.set_font( &app::res::font_default);
	dbprintf("Disp en status %i",err);
	dbprintf("LOOP>>>%i",a);
	err = disp.clear();
	dbprintf("Disp clear status %i",err);
	err = disp.puts("Ada to nie");
	dbprintf("Disp puts status %i", err );
	err = disp.endl();
	dbprintf("Disp puts status %i", err );
	err = disp.puts("Display line2");
	dbprintf("Disp puts status %i", err );
	disp.set_font( &app::res::font_big );
	err = disp.setpos(0,32);
	dbprintf("Disp puts setpos %i", err );
	err = disp.puts("1234");
	dbprintf("Disp puts status %i", err );
	err = disp.progress_bar( 32,16,80,16,50,100);
	//err = disp.draw_hline( 0, 63, 80, fnd::drv::lcd::color::black);
	err = disp.show_icon( 12,16,&app::res::manual_icon );
	dbprintf("draw hline status %i", err );
#endif
	//app::hrtim_test_init();
	spi_rwtest(m_spi);
	spi_rwtest(m_spi);
	}
	//DMA instance test
	auto& ctrl = periph::dma::controller::instance();
	for(;;) {
		isix::wait_ms(1000);
	}
}

}

auto main() -> int
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
		periph::drivers::uart_early::open,
		"serial0", 115200
	);
	isix::task_create( app::test_thread, nullptr, 1536, isix::get_min_priority() );
		dbprintf("<<<< You welcome A >>>>");
	isix::start_scheduler();
	return 0;
}
