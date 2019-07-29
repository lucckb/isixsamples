/** Example shows howto use events mechanism to the intertask
 *  communicaton
 *  Hardware platform: STM32F411E-DISCO
 *  PA2 Port - USART TXD should be connected to serial<->usb converter
 */
#include <config/conf.h>
#include <periph/gpio/gpio.hpp>
#include <foundation/sys/dbglog.h>
#include <periph/drivers/serial/uart_early.hpp>
#include <periph/drivers/i2c/i2c_master.hpp>
#include <isix.h>
#include <cstring>
#include "lms303.hpp"

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
		periph::drivers::i2c_master i2c1("i2c1");
		app::lms303 lms(i2c1);
		static constexpr auto IFC_TIMEOUT = 1000;
		do {
			int ret = i2c1.open(IFC_TIMEOUT);
			if(ret) {
				dbg_err("Unable to open i2c device error: %i", ret);
				break;
			}
			ret = i2c1.set_option(periph::option::speed(400'000));
			if(ret) {
				dbg_err("Unable to set i2c option error: %i", ret);
				break;
			}
			ret = lms.configure();
			if(ret) {
				dbg_err("LMS sensor init error: %i", ret);
				break;
			}
			for(int x,y,z;;) {
				ret = lms.get_orientation(x,y,z);
				if(ret) {
					dbg_err("LMS orientation error: %i", ret);
					return;
				}
				dbg_info("X pos: %i Y pos %i Z pos %i", x,y,z);
				isix::wait_ms(500);
			}
		} while(0);
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
    dbprintf("<<<< MEMS I2C sensor demo >>>>");
	isix::start_scheduler();
}


