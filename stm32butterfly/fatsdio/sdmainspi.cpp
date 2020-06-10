#include <isix.h>
#include <stm32lib.h>
#include <foundation/sys/dbglog.h>
#include <foundation/sys/tiny_printf.h>
#include <usart_simple.h>
#include <config/conf.h>
#include <stm32system.h>
#include <stm32rcc.h>
#include <stm32gpio.h>
#include <fs/fat.h>
#include <cctype>
#include <cstring>
#include <stm32spi.h>
#include <mmc/mmc_host_spi.hpp>
#include <mmc/immc_det_pin.hpp>
#include <mmc/mmc_slot.hpp>
#include <mmc/mmc_card.hpp>
#include <stm32gpio.h>
#include <stm32_spi_master_dma.hpp>
#include <isix/arch/irq.h>

namespace {



/** Cortex stm32 System setup
 * Clock and flash configuration for selected rate
 */
uint32_t uc_periph_setup()
{
    stm32::rcc_flash_latency( CONFIG_HCLK_HZ );
    const uint32_t freq = stm32::rcc_pll1_sysclk_setup( stm32::e_sysclk_hse_pll, CONFIG_XTAL_HZ , CONFIG_HCLK_HZ );
    stm32::rcc_pclk2_config(  RCC_HCLK_Div1 );
    stm32::rcc_pclk1_config(  RCC_HCLK_Div2 );
    //Setup NVIC vector at begin of flash
    SCB->VTOR = NVIC_VectTab_FLASH;
    return freq;
}
extern "C"
{

//! This function is called just before call global constructors
void _external_startup(void)
{

	//Initialize system perhipheral
	const uint32_t freq = uc_periph_setup();

	//1 bit for preemtion priority
	isix_set_irq_priority_group( isix_cortexm_group_pri7 );

	//Initialize isix
	isix_init(freq);

}
} /* extern C */

}
namespace app
{

class ledblink: public isix::task_base
{
public:
	//Constructor
	ledblink() :  LED_PORT(GPIOE)
	{
		using namespace stm32;
		gpio_clock_enable( LED_PORT, true);
		gpio_abstract_config(LED_PORT, LED_PIN, AGPIO_MODE_OUTPUT_PP, AGPIO_SPEED_HALF );
		start_thread( STACK_SIZE, TASK_PRIO );
	}
protected:
	//Main functionQMonikQ
	virtual void main() noexcept
	{
		while(true)
		{
			//Enable LED
			stm32::gpio_clr( LED_PORT, LED_PIN );
			//Wait time
			isix::wait_ms( BLINK_TIME );
			//Disable LED
			stm32::gpio_set( LED_PORT, LED_PIN );
			//Wait time
			isix::wait_ms( BLINK_TIME );
		}
	}
private:
	static const unsigned STACK_SIZE = 2048;
	static const unsigned TASK_PRIO = 3;
	GPIO_TypeDef * const LED_PORT;
	static const unsigned LED_PIN = 14;
	static const unsigned BLINK_TIME = 500;
};


class stm32_gpio : public drv::mmc::immc_det_pin
{
public:
	stm32_gpio()
	{
		using namespace stm32;
		gpio_clock_enable( GPIOD, true );
		gpio_abstract_config( GPIOD, 0, AGPIO_MODE_INPUT_PULLUP, AGPIO_SPEED_HALF );
	}
	virtual bool get() const
	{
		return !stm32::gpio_get( GPIOD, 0 );
	}
};



class fat_tester: public isix::task_base
{
public:
	//Constructor
	fat_tester()
		: m_spi(SPI1,CONFIG_PCLK1_HZ, CONFIG_PCLK2_HZ),
		  m_mmc_host( m_spi,11000 ), m_slot( m_mmc_host, m_pin )
	{
		start_thread( STACK_SIZE, TASK_PRIO );
	}
protected:
	//Main function
	virtual void main() noexcept
	{
		isix::wait_ms( 100 );
		static char buf[513];
		FATFS fs;
		int err;
		FIL f;
		UINT ccnt;
		disk_add( 0, &m_slot );
		for(;;)
		{
			const int cstat = m_slot.check();
			if( cstat == drv::mmc::mmc_slot::card_inserted )
			{
				std::memset(buf, 0, sizeof(buf) );
				err = f_mount(0, &fs);
				dbprintf("Fat disk mount status %i", err );
				if( !err )
				{
					int err = f_open(&f, "toread.txt", FA_READ );
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
					int err = f_open(&f, "write.txt", FA_WRITE | FA_CREATE_ALWAYS );
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
			isix::wait_ms( 100 );
		}
	}
private:
		static const unsigned STACK_SIZE = 2048;
		static const unsigned TASK_PRIO = 3;
		stm32::drv::spi_master_dma m_spi;
		drv::mmc::mmc_host_spi m_mmc_host;
		stm32_gpio m_pin;
		drv::mmc::mmc_slot m_slot;
};



class mmc_host_tester : public isix::task_base
{
public:
	mmc_host_tester()
		: m_spi(SPI1,CONFIG_PCLK1_HZ, CONFIG_PCLK2_HZ),
		  m_mmc_host( m_spi,11000 ), m_slot( m_mmc_host, m_pin )
	{
		start_thread( STACK_SIZE, TASK_PRIO );
	}
private:
	void transfer_read_test( drv::mmc::mmc_card *card, char *buf, size_t size )
	{
		const size_t N_sects = 1000;
		int ret;
		for(size_t bs=512; bs<=size; bs+=512 )
		{
			dbprintf("Read test block size %u", bs );
			ostick_t begin = isix::get_jiffies();
			for(size_t c=0; c<N_sects; c+=(bs/512) )
			{
				ret = card->read( buf, c, bs/512 );
				if( ret )
				{
					dbprintf("Read error with code %i", ret);
					return;
				}
			}
			ostick_t time = isix::get_jiffies() - begin;
			dbprintf("Speed %u kb/s", (1000*(N_sects/2)) / time);
		}
	}
	void transfer_write_test( drv::mmc::mmc_card *card, char *buf, size_t size )
	{
		const size_t N_sects = 1000;
		int ret;
		for(size_t bs=512; bs<=size; bs+=512 )
		{
			dbprintf("Write test block size %u", bs );
			ostick_t begin = isix::get_jiffies();
			for(size_t c=0; c<N_sects; c+=(bs/512) )
			{
				ret = card->write( buf, c, bs/512 );
				if( ret )
				{
					dbprintf("Write error with code %i", ret);
					return;
				}
			}
			ostick_t time = isix::get_jiffies() - begin;
			dbprintf("Speed %u kb/s", (1000*(N_sects/2)) / time);
		}
	}
protected:
	virtual void main() noexcept
	{
		for(;;)
		{
			const int cstat = m_slot.check();
			if(  cstat == drv::mmc::mmc_slot::card_inserted )
			{
				static char buf[4096] = { '\0' };
				drv::mmc::mmc_card *c;
				int ret;
				static drv::mmc::cid cid;
				dbprintf("Open %i %p", (ret=m_slot.get_card(c)), (void*)c);
				if( ret ) break;
				dbprintf("Write ret=%i",(ret=c->write("ZUPA", 7777, 3 )));
				if( ret ) break;
				//isix::isix_wait_ms(100);
				dbprintf( "Read ret=%i", (ret=c->read( buf, 7777, 1 )) );
				if( ret ) break;
				dbprintf("GOT SSTR %s", buf );
				dbprintf( "CIDST=%i",(ret=c->get_cid( cid )));
				if( ret ) break;
				dbprintf("CIDS M=%s Y=%hi M=%hi" , cid.prod_name, cid.year, cid.month );
				//Get sector count
				uint32_t sectors;
				dbprintf("SECTORS=%lu", c->get_sectors_count() );
				dbprintf("ERASESIZ=%li %lu", (ret=c->get_erase_size(sectors)), sectors );
				if( ret ) break;
				dbprintf( "Read ret=%i", (ret=c->read( buf, 7777, 1 )) );
				if( ret ) break;
				dbprintf("GOT SSTR %s", buf );
				//Check read speed
				transfer_read_test(c,buf,sizeof(buf));
				//Write test
				transfer_write_test(c, buf, sizeof(buf));
			}
			else
			{
				dbprintf("CSTAT=%i",cstat);
			}
		}
	}
private:
		static const unsigned STACK_SIZE = 2048;
		static const unsigned TASK_PRIO = 1;
		stm32::drv::spi_master_dma m_spi;
		drv::mmc::mmc_host_spi m_mmc_host;
		stm32_gpio m_pin;
		drv::mmc::mmc_slot m_slot;
};



}	//namespace app end

//App main entry point
int main()
{
	 dblog_init( stm32::usartsimple_putc, NULL, stm32::usartsimple_init,
	    		USART2,115200,true, CONFIG_PCLK1_HZ, CONFIG_PCLK2_HZ );
	 dbprintf(" Exception presentation app using ISIXRTOS");
	//The blinker class
	static app::ledblink led_blinker;
	//The ledkey class
	static app::fat_tester ft;
	//static app::mmc_host_tester ht;
	isix::wait_ms(1000);
	//Start the isix scheduler
	isix::start_scheduler();
}


