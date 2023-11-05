#include <isix.h>
#include <foundation/sys/dbglog.h>
#include <periph/gpio/gpio.hpp>
#include <periph/drivers/serial/uart_early.hpp>
#include <config/conf.h>

namespace {
    constexpr auto led_0 = periph::gpio::num::PE14;
}

namespace app {
    void test_thread(void*) {
        for(int i=0;;++i) {
            isix::wait_ms(500);
            if(i%2==0) {
                dbprintf("Loop %i",i>>1);
            }
            periph::gpio::set(led_0, i%2);
        }
    }
}


//App main entry point
int main()
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
    dbprintf("<<<< Hello ZL41ARM board >>>>");
    isix::start_scheduler();
    return 0;

}


