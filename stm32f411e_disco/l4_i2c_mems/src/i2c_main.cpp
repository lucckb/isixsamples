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



	//Task for the led blinking
	auto watch_sensor() -> void {
		periph::drivers::i2c_master i2c1("i2c1");
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
            // Fill the sensor functions
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


