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
    constexpr auto led_0 = periph::gpio::num::PD13;
	constexpr auto key_0 = periph::gpio::num::PA0;
}

namespace {

	//Task for the led blinking
	auto blink_task() -> void {
        dbprintf("Blink task started !");
		for(;;) {
            isix::wait_ms(1000);
            dbprintf("New event !");
		}
	}

	//Interrupt input config
	auto interrupt_input_config() -> void {
	}
}


extern "C" {
	//! Exti 0 vector
	void exti0_isr_vector() {
	}
}

// Start main function
auto main() -> int
{
	static constexpr auto task_siz = 1024;
	// Create the samaphore and val
	// Wait some time before startup
    isix::wait_ms(500);
    static isix::semaphore m_ulock_sem { 1, 1 };
    isix::wait_ms( 500 );
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
	// Configure interrupt line
	interrupt_input_config();
	// Create task for blinking
	static auto t1 = isix::thread_create_and_run(task_siz,
		isix::get_min_priority(),0,blink_task);
	isix::start_scheduler();
	return 0;
}

