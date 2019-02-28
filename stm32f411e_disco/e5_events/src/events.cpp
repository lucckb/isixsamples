/** Example shows howto use fifo from irq context
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
		periph::gpio::num::PC3 };
}

namespace {
	//Task for the led blinking
	auto blink_task() -> void {
		//! Blinking loop
		for(;;) {
			for( int i=0;i<4;++i) {
				periph::gpio::set(leds[i],!periph::gpio::get(keys[i]));
			}
			isix::wait_ms(10);
		}
	}
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
}


// Start main function
auto main() -> int
{
	static constexpr auto task_siz = 1024;
	io_config();
	// Wait some time before startup
    isix::wait_ms(500);
	//Event
	isix::event ev {};
	// Create task for blinking
	static auto t1 = isix::thread_create_and_run(task_siz,
		isix::get_min_priority(),0,blink_task);
	isix::start_scheduler();
	return 0;
}


