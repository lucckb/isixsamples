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
	//Main functionQMonikQ
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
		isix::isix_wait_ms( 1000 );
		bool p_card_state = false;
		static char buf[513];
		FATFS fs;
		int err;
		FIL f;
		UINT ccnt;
		for(;;)
		{
			bool card_state = stm32::drv::isix_sdio_card_driver_is_card_in_slot();
			if (  card_state && !p_card_state )
			{
				std::memset(buf, 0, sizeof(buf) );
				err = f_mount(0, &fs);
				dbprintf("Fat disk mount status %i", err );
				//Print card info
				{
					stm32::drv::isix_sdio_card_driver_init();
					stm32::drv::sdcard_info info;
					int err = stm32::drv::isix_sdio_card_driver_get_info(&info, stm32::drv::sdcard_info_f_info );
					dbprintf("Status = %i Card info blocksize: %lu capacity %u type: %i", err,
							info.CardBlockSize, (unsigned)info.CardCapacity/1024/1024, info.CardType );
				}
				if( !err )
				{
					int err = f_open(&f, "dupa.txt", FA_READ );
					dbprintf( "Open file for read status=%i", err );
					if( !err )
					{
						for(;;)
						{
							err = f_read(&f, buf,sizeof(buf)-1,&ccnt );
							if( err || ccnt == 0 ) break;
							if( ccnt > 0 )
							{
								buf[ccnt] = '\0';
								fnd::tiny_printf("%s", buf);
							}
						}
						fnd::tiny_printf("\r\n");
						dbprintf("Read finished ret=%i count=%u", err, ccnt );
					}
					err = f_close( &f );
					dbprintf( "Close status %i", err );
				}
				if(!err)
				{
					int err = f_open(&f, "dupa.txt", FA_WRITE | FA_CREATE_ALWAYS );
					dbprintf( "Open file for write status=%i", err );
					if(!err )
					{
						static const char wstr[] = "Ala ma kota a kot ma ale\r\n";
						err = f_write( &f, wstr, sizeof(wstr)-1, &ccnt );
						dbprintf("Write finished ret=%i count=%u", err, ccnt );
						err = f_close( &f );
						dbprintf( "Close status %i", err );
					}
				}
			}
			p_card_state = card_state;
			isix::isix_wait_ms( 100 );
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

