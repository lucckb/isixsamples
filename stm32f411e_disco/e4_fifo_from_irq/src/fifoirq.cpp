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
	auto blink_task(isix::fifo<bool>& fifo) -> void {
		// Configure gpio as output
		periph::gpio::setup( led_0,
			periph::gpio::mode::out{
				periph::gpio::outtype::pushpull,
				periph::gpio::speed::low
		} );
		//! Blinking loop
		for(;;) {
			// Wait for the semaphore read the variable
			bool state {};
			fifo.pop(state,ISIX_TIME_INFINITE);
			periph::gpio::set(led_0,state);
		}
	}

	//Interrupt input config
	auto interrupt_input_config() -> void {
		// Configure gpio as input
		periph::gpio::setup(key_0,
			periph::gpio::mode::in{periph::gpio::pulltype::down}
		);
		//Configure EXTI controller
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
		LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);
		LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_0);
		LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_0);
		//Nvic enable irq
		isix::set_irq_priority(EXTI0_IRQn, {1, 7});
		isix::request_irq(EXTI0_IRQn);
	}
}

namespace {
	//global fifo
	isix::fifo<bool> fifo { 20 };
}

extern "C" {
	//! Exti 0 vector
	void exti0_isr_vector() {
		//Debounce time and debounce timer
		static constexpr auto debounce_ms {100};
		static ostick_t last_irq_tick {};
		//Current state on or off
		static bool state {};
		//If exti0 clear flag and check usec timer
		if(LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_0)) {
			LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
			//If debounce time  change state
			if(isix::timer_elapsed(last_irq_tick,debounce_ms)) {
				state = !state;
				fifo.push_isr(state);
			}
			last_irq_tick = isix::get_jiffies();
		}
	}
}

// Start main function
auto main() -> int
{
	static constexpr auto task_siz = 1024;
	// Create the samaphore and val
	// Wait some time before startup
    isix::wait_ms(500);
	// Configure interrupt line
	interrupt_input_config();
	// Create task for blinking
	static auto t1 = isix::thread_create_and_run(task_siz,
		isix::get_min_priority(),0,blink_task,std::ref(fifo));
	isix::start_scheduler();
	return 0;
}


