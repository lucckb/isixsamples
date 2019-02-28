/** Example shows howto use mutexes
 *  Hardware platform: STM32F411E-DISCO
 *  PA2 Port - USART TXD should be connected to serial<->usb converter
 */
#include <config/conf.h>
#include <periph/gpio/gpio.hpp>
#include <isix.h>



namespace {
    constexpr auto led_0 = periph::gpio::num::PD13;
	constexpr auto key_0 = periph::gpio::num::PA0;
}

namespace {
	// Task for button scanning
	auto keyb_scan_task( isix::thread& task_ctl ) -> void {
		// Configure gpio as input
		periph::gpio::setup(key_0,
			periph::gpio::mode::in{periph::gpio::pulltype::down}
		);
		// Suspend resume flag
		bool resume {};
		for(auto pstate=false;;) {
			//Get key
			const auto val = periph::gpio::get(key_0);
			// On rising edge change state
			if(val && !pstate) {
				resume = !resume;
				//Suspend or resume task
				if(resume) task_ctl.resume();
				else task_ctl.suspend();
			}
			//Save previous state
			pstate = val;
			isix::wait_ms(10);
		}
	}
	//Task for the led blinking
	auto blink_task() -> void {
		// Configure gpio as output
		periph::gpio::setup( led_0,
			periph::gpio::mode::out{
				periph::gpio::outtype::pushpull,
				periph::gpio::speed::low
		} );
		//! Blinking loop
		for(;;) {
			periph::gpio::toggle(led_0);
			isix::wait_ms(500);
		}
	}
}

auto main() -> int
{
	// Wait some time before startup
    isix::wait_ms( 500 );
	// Create task for blinking
	static auto t1 = isix::thread_create_and_run(ISIX_MIN_STACK_SIZE,
		isix::get_min_priority(), isix_task_flag_suspended, blink_task );
	// Create task 2
	static auto t2 = isix::thread_create_and_run(ISIX_MIN_STACK_SIZE,
		isix::get_min_priority(),0, keyb_scan_task, std::ref(t1));
	isix::start_scheduler();
	return 0;
}

