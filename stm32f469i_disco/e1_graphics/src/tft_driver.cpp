#include <config/conf.h>
#include <isix.h>
#include <foundation/sys/dbglog.h>
#include <periph/drivers/serial/uart_early.hpp>
#include <periph/gpio/gpio.hpp>
#include <gfx/gui/label.hpp>



namespace {
    constexpr auto led_0 = periph::gpio::num::PD5;
}

namespace {
	void sdram_memtest(void)
	{
		const auto addr = reinterpret_cast<volatile uint32_t*>(0xc0000000);
		constexpr auto mem_siz = 4U * 1024U * 1024U;
		bool fail {};
		//! Index pattern
		for (auto i=0U; i<mem_siz; i++) {
			*(addr + i) = i;
		}
		for (auto i=0U; i<mem_siz; i++) {
			if(*(addr + i)!=i) {
				dbprintf("sdram test1 failed <%08x>@%p ", *(addr + i), addr+i );
				fail = true;
				break;
			}
		}
		//0x55 test
		for (auto i=0U; i<mem_siz; i++) {
			*(addr + i) = 0x5555'5555U;
		}
		for (auto i=0U; i<mem_siz; i++) {
			if(*(addr + i)!=0x5555'5555U) {
				dbprintf("sdram test2 failed <%08x>@%p ", *(addr + i), addr+i );
				fail = true;
				break;
			}
		}
		if(!fail)
			dbprintf("sdram test completed OK");
	}

}

namespace app {
    void test_thread(void*) {
		sdram_memtest();
        for(int i=0;;++i) {
            isix::wait_ms(500);
            if(i%2==0) {
                dbprintf("Loop %i",i>>1);
            }
            periph::gpio::set(led_0, i%2);
        }
    }
}


auto main() -> int
{
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
    // Configure PD13 pin LED as an output
    periph::gpio::setup( led_0,
        periph::gpio::mode::out{
            periph::gpio::outtype::pushpull,
            periph::gpio::speed::low
        }
    );
	isix::task_create( app::test_thread, nullptr, 1536, isix::get_min_priority() );
    dbprintf("<<<< Hello STM32F411E-DISCO TFT-DEMO >>>>");
	isix::start_scheduler();
	return 0;
}
