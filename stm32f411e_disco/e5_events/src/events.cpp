/** Example shows howto use events mechanism to the intertask
 *  communicaton
 *  Hardware platform: STM32F411E-DISCO
 *  PA2 Port - USART TXD should be connected to serial<->usb converter
 */
#include <config/conf.h>
#include <periph/gpio/gpio.hpp>
#include <foundation/sys/dbglog.h>
#include <periph/drivers/serial/uart_early.hpp>
#include <isix.h>
#include <isix/arch/irq_platform.h>
#include <isix/arch/irq.h>
#include <stm32_ll_exti.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_system.h>



namespace {
	// Output LEDS
    constexpr int leds[] =  { periph::gpio::num::PD0,
		periph::gpio::num::PD1,periph::gpio::num::PD2,
		periph::gpio::num::PD3 };
	//Input keys
	constexpr int keys[] = { periph::gpio::num::PC0,
		periph::gpio::num::PC1,periph::gpio::num::PC2,
		periph::gpio::num::PC3, periph::gpio::num::PC4,
	    periph::gpio::num::PC5
	};
}

namespace {
	//IO pins conf
	auto io_config() -> void {
		// Configure gpios as input
		for(auto k: keys) {
			periph::gpio::setup(k,
				periph::gpio::mode::in{periph::gpio::pulltype::up}
			);
		}
		// Configure leds as output
		for(auto l: leds) {
			periph::gpio::setup( l,
				periph::gpio::mode::out{
					periph::gpio::outtype::pushpull,
					periph::gpio::speed::low
			} );
		}
	}

	// Task for button scanning and semaphore signaling
	auto key_scan_task(isix::event& ev, int id) -> void {
		// Suspend resume flag
		for(auto pstate=true;;) {
			//Get key
			const auto val = periph::gpio::get(keys[id]);
			// On rising edge change state
			if(!val && pstate) {
				ev.set( 1U<<id );
			}
			//Save previous state
			pstate = val;
			isix::wait_ms(10);
		}
	}

	//Task for the led blinking
	auto blink_task(isix::event& ev, int led, unsigned wait_for ) -> void {
		//! Blinking loop
		for(;;) {
			ev.wait(wait_for, true, true);
			periph::gpio::toggle(leds[led]);
		}
	}
}


// Start main function
auto main() -> int
{
	io_config();
	// Wait some time before startup
    isix::wait_ms(500);
	//Event
	static isix::event ev {};
	// Create 4 tasks for led controlling
	static isix::thread ledtasks[] = {
		isix::thread_create_and_run(ISIX_MIN_STACK_SIZE,
			isix::get_min_priority(),0,blink_task, std::ref(ev), 0, 0x01 ),
		isix::thread_create_and_run(ISIX_MIN_STACK_SIZE,
			isix::get_min_priority(),0,blink_task, std::ref(ev), 1, 0x02 ),
		isix::thread_create_and_run(ISIX_MIN_STACK_SIZE,
			isix::get_min_priority(),0,blink_task, std::ref(ev), 2, 0x04 ),
		isix::thread_create_and_run(ISIX_MIN_STACK_SIZE,
			isix::get_min_priority(),0,blink_task, std::ref(ev), 3, 0x18 ),
	};
	//! Create 5 tasks for keyboard scanning
	static isix::thread scantsk[] = {
		isix::thread_create_and_run(ISIX_MIN_STACK_SIZE,
			isix::get_min_priority(),0,key_scan_task,std::ref(ev), 0),
		isix::thread_create_and_run(ISIX_MIN_STACK_SIZE,
			isix::get_min_priority(),0,key_scan_task,std::ref(ev), 1),
		isix::thread_create_and_run(ISIX_MIN_STACK_SIZE,
			isix::get_min_priority(),0,key_scan_task,std::ref(ev), 2),
		isix::thread_create_and_run(ISIX_MIN_STACK_SIZE,
			isix::get_min_priority(),0,key_scan_task,std::ref(ev), 3),
		isix::thread_create_and_run(ISIX_MIN_STACK_SIZE,
			isix::get_min_priority(),0,key_scan_task,std::ref(ev), 4),
	};
	isix::start_scheduler();
}


