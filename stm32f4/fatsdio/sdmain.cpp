#include <isix.h>
#include <stm32lib.h>
#include <dbglog.h>
#include <usart_simple.h>
#include "config.hpp"
#include <stm32system.h>
#include <stm32rcc.h>
#include <stm32gpio.h>
#include <stm32tim.h>
#include <stm32adc.h>
#include <stm32dma.h>
#include <stm32pwr.h>
#include <fs/fat.h>
#include <stm32sdio.h>
#include "sdio_sdcard_driver.h"
#include <cctype>
#include <cstring>
/* ------------------------------------------------------------------ */
namespace {
/* ------------------------------------------------------------------ */

//Number of isix threads
const unsigned ISIX_NUM_PRIORITIES = 4;
//SysTimer values
const unsigned MHZ = 1000000;

/* ------------------------------------------------------------------ */
/** Cortex stm32 System setup
 * Clock and flash configuration for selected rate
 */
void uc_periph_setup()
{
    stm32::rcc_flash_latency( config::HCLK_HZ );
    stm32::rcc_pll1_sysclk_setup( stm32::e_sysclk_hse_pll, config::XTAL_HZ , config::HCLK_HZ );
    stm32::rcc_pclk2_config(  RCC_HCLK_Div2 );
    stm32::rcc_pclk1_config(  RCC_HCLK_Div4 );
    //Setup NVIC vector at begin of flash
    SCB->VTOR = NVIC_VectTab_FLASH;
}

/* ------------------------------------------------------------------ */
extern "C"
{

/* ------------------------------------------------------------------ */
//! This function is called just before call global constructors
void _external_startup(void)
{

	//Initialize system perhipheral
	uc_periph_setup();

	//1 bit for preemtion priority
	stm32::nvic_priority_group(NVIC_PriorityGroup_1);

	//System priorities
	stm32::nvic_set_priority(PendSV_IRQn,1,0x7);

	//System priorities
	stm32::nvic_set_priority(SVCall_IRQn,1,0x7);

	//Set timer priority
	stm32::nvic_set_priority(SysTick_IRQn,1,0x7);

	//Initialize isix
	isix::isix_init(ISIX_NUM_PRIORITIES);

	stm32::systick_config( isix::ISIX_HZ * (config::HCLK_HZ/(8*MHZ)) );
}
/* ------------------------------------------------------------------ */
} /* extern C */

/* ------------------------------------------------------------------ */
}
/* ------------------------------------------------------------------ */
namespace app
{

/* ------------------------------------------------------------------ */
class ledblink: public isix::task_base
{
public:
	//Constructor
	ledblink() : task_base(STACK_SIZE,TASK_PRIO), LED_PORT(GPIOE)
	{
		using namespace stm32;
		gpio_clock_enable( LED_PORT, true);
		gpio_abstract_config(LED_PORT, LED_PIN, AGPIO_MODE_OUTPUT_PP, AGPIO_SPEED_HALF );
	}
protected:
	//Main function
	virtual void main()
	{
		while(true)
		{
			//Enable LED
			stm32::gpio_clr( LED_PORT, LED_PIN );
			//Wait time
			isix::isix_wait( isix::isix_ms2tick(BLINK_TIME) );
			//Disable LED
			stm32::gpio_set( LED_PORT, LED_PIN );
			//Wait time
			isix::isix_wait( isix::isix_ms2tick(BLINK_TIME) );
		}
	}
private:
	static const unsigned STACK_SIZE = 2048;
	static const unsigned TASK_PRIO = 3;
	GPIO_TypeDef * const LED_PORT;
	static const unsigned LED_PIN = 14;
	static const unsigned BLINK_TIME = 500;
};
/* ------------------------------------------------------------------ */

class ledkey: public isix::task_base
{
public:
	//Constructor
	ledkey()
		: task_base(STACK_SIZE,TASK_PRIO)
	{
	}
protected:
	//Main function
	virtual void main()
	{
		//for(;;)
		if(1)
		{
			dbprintf("SDINIT status %i", stm32::drv::isix_sdio_card_driver_init() );
			isix::isix_wait_ms( 1000 );
			while(1)
			{
				if ( !(stm32::drv::isix_sdio_card_driver_status() & stm32::drv::SDCARD_DRVSTAT_NODISK) )
				{
					dbprintf("REINIT status %i", stm32::drv::isix_sdio_card_driver_reinitialize());
					break;
				}
				isix::isix_wait_ms( 1000 );
			}
			{
				stm32::drv::sdcard_info info;
				stm32::drv::isix_sdio_card_driver_get_info( &info );
				dbprintf("SDCS FREQ %i", info.SD_csd.MaxBusClkFrec );
			}
			static char buf[512];
			std::strcpy(buf,"Ala ma kota");
			const int wr = stm32::drv::isix_sdio_card_driver_write( buf, 20000, 1 );
			dbprintf("Wr status=%d", wr);
			std::memset(buf, 0, sizeof(buf) );
			const int rd = stm32::drv::isix_sdio_card_driver_read(  buf, 20000 , 1 );
			dbprintf("Rd status=%d", rd);
			for(int i=0; i<512; i++ )
			{
				fnd::tiny_printf("%c", std::isprint(buf[i])?buf[i]:'.');
			}
			fnd::tiny_printf("\r\n");
		}
		else
		{
			FATFS fs;
			static char buf[512];
			dbprintf("FMNTSTAT=%i NETR=%hi", f_mount(0, &fs), fs.n_rootdir );
			FIL f;
			dbprintf( "OPENRES=%i", f_open(&f, "kupa.txt", FA_READ ) );
			UINT rlen;
			dbprintf( "RES=%i RL=%i", f_read( &f, buf, sizeof(buf), &rlen ), rlen );
		}
	}
private:
		static const unsigned STACK_SIZE = 2048;
		static const unsigned TASK_PRIO = 3;
};

/* ------------------------------------------------------------------ */

}	//namespace app end
/* ------------------------------------------------------------------ */
//App main entry point
int main()
{
	 dblog_init( stm32::usartsimple_putc, NULL, stm32::usartsimple_init,
	    		USART2,115200,true, config::PCLK1_HZ, config::PCLK2_HZ );
	 dbprintf(" Exception presentation app using ISIXRTOS ");
	//The blinker class
	static app::ledblink led_blinker;
	//The ledkey class
	static app::ledkey led_key;
	isix::isix_wait_ms(1000);
	//Start the isix scheduler
	isix::isix_start_scheduler();
}

/* ------------------------------------------------------------------ */

