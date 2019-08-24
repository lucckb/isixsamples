#include <config/conf.h>
#include <isix.h>
#include <foundation/sys/dbglog.h>
#include <periph/drivers/serial/uart_early.hpp>
#include <periph/gpio/gpio.hpp>
#include <gfx/drivers/disp/dsi_fb.hpp>
#include <periph/drivers/display/bus/dsi.hpp>
#include <periph/drivers/display/rgb/fbdev.hpp>
#include <periph/drivers/display/rgb/otm8009a.hpp>
#include <stm32_ll_rcc.h>
#include <stm32_ll_bus.h>



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
		const auto t1 = isix::get_ujiffies();
		for (auto i=0U; i<mem_siz; i++) {
			*(addr + i) = i;
		}
		const auto t2 = isix::get_ujiffies();
		for (auto i=0U; i<mem_siz; i++) {
			if(*(addr + i)!=i) {
				dbprintf("sdram test1 failed <%08x>@%p ", *(addr + i), addr+i );
				fail = true;
				break;
			}
		}
		const auto t3 = isix::get_ujiffies();
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
		if(!fail) {
			dbprintf("sdram test completed OK");
			const auto wr_kbs = ((t2-t1)*(mem_siz/1024))/1000000;
			const auto rd_kbs = ((t3-t2)*(mem_siz/1024))/1000000;
			dbprintf("Write speed %i kb/s Read speed %i kb/s", wr_kbs,rd_kbs);
		}
	}

	void gdi_tester() {

		periph::display::bus::dsi dsi { "dsi" };
		periph::display::fbdev fb { "ltdc" };
		periph::display::otm8009a displl {dsi, "display"};
		gfx::drv::dsi_fb disp { fb,displl };
		disp.power_ctl( gfx::drv::power_ctl_t::on);
		disp.backlight(0);
		disp.clear(gfx::color::Tomato);
	}

}

extern "C" {
	void app_init();
}

namespace app {
    void test_thread(void*) {
		sdram_memtest();
		gdi_tester();
        for(int i=0;;++i) {
            isix::wait_ms(500);
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
	isix::task_create( ::app::test_thread, nullptr, 1536, isix::get_min_priority() );
    dbprintf("<<<< Hello STM32F411E-DISCO TFT-DEMO >>>>");
	isix::start_scheduler();
	return 0;
}
