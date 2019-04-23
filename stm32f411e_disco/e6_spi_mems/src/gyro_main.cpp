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
#include <periph/dma/dma.hpp>
#include <cstring>

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

	class leds_vis {
	public:
		enum ledx {
			none,c1fwd, c1bk, c2fwd, c2bk
		};
		//! Leds setup constructor
		leds_vis() {
			using namespace periph;
			for(int p=gpio::num::PD12;p<=gpio::num::PD15;++p) {
				gpio::setup(p,
					gpio::mode::out {gpio::outtype::pushpull,gpio::speed::low}
				);
			}
		}
		//Leds setup destructor
		void operator()(ledx arg) {
			using namespace periph;
			switch(arg) {
				case c1fwd:
					gpio::set(gpio::num::PD13,true);
					gpio::set(gpio::num::PD15,false);
					break;
				case c1bk:
					gpio::set(gpio::num::PD13,false);
					gpio::set(gpio::num::PD15,true);
					break;
				case c2fwd:
					gpio::set(gpio::num::PD12,true);
					gpio::set(gpio::num::PD14,false);
					break;
				case c2bk:
					gpio::set(gpio::num::PD12,false);
					gpio::set(gpio::num::PD14,true);
					break;
				case none:
					gpio::set(gpio::num::PD12,false);
					gpio::set(gpio::num::PD13,false);
					gpio::set(gpio::num::PD14,false);
					gpio::set(gpio::num::PD15,false);
					break;

			}
		}
	};



	//Task for the led blinking
	auto watch_sensor() -> void {
		namespace opt = periph::option;
		int ret {};
		periph::drivers::spi_master spi("spi1");
		app::l3gd20 sensor(spi,0);
		leds_vis leds;
		do {
			if((ret=spi.set_option(opt::speed(10E6)))<0) {
				break;
			}
			if((ret=spi.set_option(opt::polarity(opt::polarity::low)))<0) {
				break;
			}
			if((ret=spi.set_option(opt::phase(opt::phase::_1_edge)))<0) {
				break;
			}
			if((ret=spi.set_option(opt::dwidth(8)))<0) {
				break;
			}
			if((ret=spi.set_option(opt::bitorder(opt::bitorder::msb)))<0) {
				break;
			}
			if((ret=spi.open(ISIX_TIME_INFINITE))<0) {
				break;
			}
			app::l3gd20::value_t val;
			if((ret=sensor.enable(true))!=0) {
				dbg_info("Sensor en fail %i",ret);
				return;
			}
			for(;;) {
				val = {};
				if((ret=sensor.read(val))<0) break;
				if( val.x > 500 ) {
					leds(leds_vis::c1fwd);
				} else if( val.x < -500 ) {
					leds(leds_vis::c1bk);
				} else if( val.y > 500 ) {
					leds(leds_vis::c2fwd);
				} else if( val.y < -500 ) {
					leds(leds_vis::c2bk);
				} else {
					leds(leds_vis::none);
				}
				dbg_info("Gyro x: %i y: %i z: %i",val.x,val.y,val.z);
				isix::wait_ms(1000);
			}
		} while(0);
		dbg_info("Task failed finished with code %i", ret);
	}

	}


// Start main function
auto main() -> int
{
	usart_protected_init();
	// Wait some time before startup
    isix::wait_ms(500);
	static constexpr auto tsk_stack = 1024;
	static constexpr auto tsk_prio = 6;
	static auto watch_task =
		isix::thread_create_and_run(tsk_stack,tsk_prio,0,watch_sensor);
    dbprintf("<<<< MEMS sensor demo >>>>");
	isix::start_scheduler();
}


