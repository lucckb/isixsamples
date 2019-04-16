/** Example shows howto use events mechanism to the intertask
 *  communicaton
 *  Hardware platform: STM32F411E-DISCO
 *  PA2 Port - USART TXD should be connected to serial<->usb converter
 */
#include <config/conf.h>
#include <periph/gpio/gpio.hpp>
#include <foundation/sys/dbglog.h>
#include <periph/drivers/serial/uart_early.hpp>
#include <periph/drivers/spi/spi_master.hpp>
#include <isix.h>
#include "l3gd20.hpp"

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


	//Task for the led blinking
	auto watch_sensor() -> void {
		namespace opt = periph::option;
		int ret;
		periph::drivers::spi_master m_spi("spi1");
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
		ret = m_spi.open(ISIX_TIME_INFINITE);
		dbprintf("SPI open status %i", ret);
		app::l3gd20 sensor(m_spi,0);
		app::l3gd20::value_t val;
		app::l3gd20::temp_t temp;
		if((ret=sensor.enable(true))<0) {
			dbprintf("Initialize failure %i",ret);
			return;
		}
		for(;;) {
			do {
				temp = {};
				val = {};
				if((ret=sensor.read_temp(temp))<0) break;
				if((ret=sensor.read(val))<0) break;
			} while(0);
			if(!ret) {
				dbprintf("Temp %i", temp );
				dbprintf("x %i y %i z %i", val.x, val.y, val.z );
			} else {
				dbprintf("Ret status %i",ret);
			}
			isix::wait_ms(1000);
		}
	}
}


// Start main function
auto main() -> int
{
	usart_protected_init();
	// Wait some time before startup
    isix::wait_ms(500);
	//Event
	static constexpr auto tsk_stack = 1024;
	static constexpr auto tsk_prio = 6;
	static auto watch_task =
		isix::thread_create_and_run(tsk_stack,tsk_prio,0,watch_sensor);
    dbprintf("<<<< MEMS sensor demo >>>>");
	isix::start_scheduler();
}


