/** Example shows howto use semaphores for interasks
 *  Hardware platform: STM32F411E-DISCO
 *  PA2 Port - USART TXD should be connected to serial<->usb converter
 */
#include <config/conf.h>
#include <periph/gpio/gpio.hpp>
#include <foundation/sys/dbglog.h>
#include <periph/drivers/serial/uart_early.hpp>
#include <isix.h>



namespace {
    constexpr auto led_0 = periph::gpio::num::PD13;
	constexpr auto key_0 = periph::gpio::num::PA0;
}

namespace {

	// Task for button scanning and semaphore signaling
	auto keyb_scan_task(isix::semaphore& sem,bool& state) -> void {
		// Configure gpio as input
		periph::gpio::setup(key_0,
			periph::gpio::mode::in{periph::gpio::pulltype::down}
		);
		// Suspend resume flag
		for(auto pstate=false;;) {
			//Get key
			const auto val = periph::gpio::get(key_0);
			// On rising edge change state
			if(val && !pstate) {
				state = !state;
				// Semaphore signal
				sem.signal();
			}
			//Save previous state
			pstate = val;
			isix::wait_ms(10);
		}
	}
	//Task for the led blinking
	auto blink_task(isix::semaphore& sem,bool& state) -> void {
		// Configure gpio as output
		periph::gpio::setup( led_0,
			periph::gpio::mode::out{
				periph::gpio::outtype::pushpull,
				periph::gpio::speed::low
		} );
		//! Blinking loop
		for(;;) {
			// Wait for the semaphore read the variable
			sem.wait(ISIX_TIME_INFINITE);
			periph::gpio::set(led_0,state);
		}
	}
}

// Start main function
auto main() -> int
{
	// Create the samaphore and val
	static isix::semaphore sem { 0, 1 };
	static bool val {};
	// Wait some time before startup
    isix::wait_ms( 500 );
	// Create task for blinking
	static auto t1 = isix::thread_create_and_run(1024,
		isix::get_min_priority(),0,blink_task,std::ref(sem),std::ref(val));
	// Create task 2
	static auto t2 = isix::thread_create_and_run(1024,
		isix::get_min_priority(),0,keyb_scan_task,std::ref(sem),std::ref(val));
	isix::start_scheduler();
	return 0;
}
