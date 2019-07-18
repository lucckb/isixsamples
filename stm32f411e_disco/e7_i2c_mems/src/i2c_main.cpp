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
		periph::drivers::i2c_master i2cm("i2c2");
		for(;;) {
			dbprintf("Tak tak ");
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
	static constexpr auto tsk_stack = 1024;
	static constexpr auto tsk_prio = 6;
	static auto watch_task =
		isix::thread_create_and_run(tsk_stack,tsk_prio,0,watch_sensor);
    dbprintf("<<<< MEMS I2C sensor demo >>>>");
	isix::start_scheduler();
}


