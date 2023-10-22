/** Example shows howto use mutexes
 *  Hardware platform: STM32F411E-DISCO
 *  PA2 Port - USART TXD should be connected to serial<->usb converter
 */
#include <config/conf.h>
#include <foundation/sys/dbglog.h>
#include <periph/drivers/serial/uart_early.hpp>
#include <periph/gpio/gpio.hpp>
#include <isix.h>


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
}


auto main() -> int
{
	static constexpr auto tsk_stk_size = 1024;
	static constexpr auto tsk_prio = 1;
    isix::wait_ms( 500 );
	usart_protected_init();
    dbprintf("<<<< Example 1. Mutexes >>>>");
	// Create task as a lambda function
	auto task_fn =
		[]( int dbonly(tsk_no) ) {
			for(;;) {
				dbprintf("Hello from task %i",tsk_no);
				isix::wait_ms(500);
			}
	};
	// Create task 1
	static auto t1 = isix::thread_create_and_run(tsk_stk_size, tsk_prio, 0, task_fn, 1 );
	// Create task 2
	static auto t2 = isix::thread_create_and_run(tsk_stk_size, tsk_prio, 0, task_fn, 2 );
	// Create task 3
	static auto t3 = isix::thread_create_and_run(tsk_stk_size, tsk_prio, 0, task_fn, 3 );
	isix::start_scheduler();
	return 0;
}

