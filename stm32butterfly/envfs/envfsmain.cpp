#include <isix.h>
#include <config.h>
#include <foundation/sys/dbglog.h>
#include <usart_simple.h>
#include <stm32system.h>
#include <isixdrv/i2c_bus.hpp>
#include "blinker.hpp"
#include "app_env.hpp"
  
namespace app {
namespace {
  
class env_tester : private isix::task_base {
	static const unsigned STACK_SIZE = 512;
	static const unsigned TASK_PRIO = 3;
	static constexpr auto envid_test = 1;
public:
	env_tester() {
		start_thread( STACK_SIZE, TASK_PRIO );
	}
protected:
	virtual void main() noexcept {
		initenv( m_i2c );
		//! Set the env
		static constexpr int val = 0x12345678;
		auto err = setenv( envid_test, val );
		dbprintf("Setenv code %i", err );
		//! Read the env
		isix::wait_ms( 2000 );
		int val2 {};
		err = getenv( envid_test, val2 );
		dbprintf("Getenv code %i value %08x", err, val2 );
	}
private:
	stm32::drv::i2c_bus m_i2c { stm32::drv::i2c_bus::busid::i2c1 , 400000 };
};
  
}}
 
//App main entry point
int main()
{
	dblog_init( stm32::usartsimple_putc, NULL, stm32::usartsimple_init,
	    		USART2,115200, true, CONFIG_PCLK1_HZ, CONFIG_PCLK2_HZ );

	//! Led blinker class
	static app::ledblink blink;
	//! Env tester
	static app::env_tester etest;
	dbprintf("Env FS test");
	//Start the isix scheduler
	isix::start_scheduler();
	dbprintf("Scheduler exit");
}

 

