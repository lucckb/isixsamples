#include <isix.h>
#include <stm32lib.h>
#include <dbglog.h>
#include <usart_simple.h>
#include "config.h"
#include <stm32system.h>
#include <stm32rcc.h>
#include <stm32gpio.h>
#include <cctype>
#include <cstring>
#include <stm32gpio.h>
#include <stm32tim.h>
#include <gfx/drivers/disp/ili9341.hpp>

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
/* ------------------------------------------------------------------ */
extern "C"
{

/* ------------------------------------------------------------------ */
//! This function is called just before call global constructors
void _external_startup(void)
{

	//Initialize system perhipheral
	const uint32_t freq = uc_periph_setup();

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

	stm32::systick_config( isix::ISIX_HZ * (freq/(8*MHZ)) );
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
class ili_gpio_bus : public gfx::drv::disp_bus
{
	static constexpr auto CSL_BIT_CMD = 0;
	static constexpr auto RS_BIT_CMD = 1;
	static constexpr auto RST_BIT_CMD = 2;
private:
	static constexpr auto DATA_PORT = GPIOE;
	static constexpr auto DATA_MASK = 0xff;
	static constexpr auto CTL_PORT = GPIOC;
	static constexpr auto CS_PIN =  4;
	static constexpr auto RS_PIN =  5;
	static constexpr auto WR_PIN =  7;
	static constexpr auto RD_PIN =  8;
	static constexpr auto RST_PIN = 9;
	static constexpr unsigned bv( unsigned pin )
	{
		return 1<<pin;
	}
	enum class dir_t : bool
	{
		in,
		out
	};

	void bus_dir( dir_t dir )
	{
		using namespace stm32;
		if( dir==dir_t::out && m_dir==dir_t::in )
		{
			gpio_abstract_config_ext( DATA_PORT, DATA_MASK, AGPIO_MODE_OUTPUT_PP, AGPIO_SPEED_FULL );
		}
		else if( dir==dir_t::in && m_dir==dir_t::out  )
		{
			gpio_abstract_config_ext( DATA_PORT, DATA_MASK, AGPIO_MODE_INPUT_PULLUP, AGPIO_SPEED_FULL );
		}
		m_dir = dir;
	}
public:
	ili_gpio_bus()
	{
		using namespace stm32;
		gpio_clock_enable( DATA_PORT, true );
		gpio_clock_enable( CTL_PORT, true );
		gpio_abstract_config_ext( DATA_PORT, DATA_MASK, AGPIO_MODE_INPUT_PULLUP, AGPIO_SPEED_FULL );
		gpio_abstract_config_ext( CTL_PORT,
			bv(CS_PIN)|bv(RS_PIN)|bv(WR_PIN)|bv(RD_PIN)|bv(RST_PIN), AGPIO_MODE_OUTPUT_PP, AGPIO_SPEED_FULL);
		gpio_set_mask( CTL_PORT, bv(CS_PIN)|bv(WR_PIN)|bv(RD_PIN)|bv(RST_PIN) );
	}
	virtual ~ili_gpio_bus() {}
	// Lock bus and set address
	virtual void set_ctlbits( int bit, bool val )
	{
		//dbprintf("ctl(bit=%i val=%i)", bit, val );
		using namespace stm32;
		const auto io_fn = val?(&gpio_set):(&gpio_clr);
		switch( bit )
		{
		case CSL_BIT_CMD: io_fn( CTL_PORT, CS_PIN ); break;
		case RS_BIT_CMD:  io_fn( CTL_PORT, RS_PIN ); break;
		case RST_BIT_CMD: io_fn( CTL_PORT, RST_PIN ); break;
		}
	}
	/* Read transfer */
	virtual void read( void *buf, std::size_t len )
	{
		using namespace stm32;
		bus_dir( dir_t::in );
		for( size_t l=0; l<len; ++l )
		{
			gpio_clr( CTL_PORT, RD_PIN );
			nop();
			*(reinterpret_cast<uint8_t*>(buf)+l) = gpio_get_mask( DATA_PORT, DATA_MASK );
			gpio_set( CTL_PORT, RD_PIN );
		}
	}
	/* Write transfer */
	virtual void write( const void *buf, size_t len )
	{
		using namespace stm32;
		bus_dir( dir_t::out );
		for( size_t l=0; l<len; ++l )
		{
			//One write
			gpio_set_clr_mask( DATA_PORT ,*(reinterpret_cast<const uint8_t*>(buf)+l), DATA_MASK );
			gpio_clr( CTL_PORT, WR_PIN );	//WR down
			gpio_set( CTL_PORT, WR_PIN );	//WR up
		}
	}
	virtual void write(uint16_t v)
	{
		using namespace stm32;
		gpio_set_clr_mask( DATA_PORT ,v>>8, DATA_MASK );
		gpio_clr( CTL_PORT, WR_PIN );	//WR down
		gpio_set( CTL_PORT, WR_PIN );	//WR up
		gpio_set_clr_mask( DATA_PORT ,v, DATA_MASK );
		gpio_clr( CTL_PORT, WR_PIN );	//WR down
		gpio_set( CTL_PORT, WR_PIN );	//WR up
	}
	/* Wait ms long delay */
	virtual void reset( )
	{
		stm32::gpio_set( CTL_PORT, RST_PIN );
		isix::isix_wait_ms( 10 );
		stm32::gpio_clr( CTL_PORT, RST_PIN );
		isix::isix_wait_ms( 10 );
		stm32::gpio_set( CTL_PORT, RST_PIN );
		isix::isix_wait_ms( 150 );
	}
private:
	dir_t m_dir { dir_t::in };
};
/* ------------------------------------------------------------------ */

class tft_tester: public isix::task_base
{
public:
	//Constructor
	tft_tester()
		: task_base(STACK_SIZE,TASK_PRIO),
		  gdisp( gbus )
	{
		//Configure and init TFT backlight
		stm32::gpio_clock_enable( GPIOC, true );
		stm32::rcc_apb2_periph_clock_cmd(RCC_APB2Periph_AFIO, true );
		stm32::rcc_apb1_periph_clock_cmd( RCC_APB1Periph_TIM3 , true );
		stm32::gpio_abstract_config( GPIOC, 6,  stm32::AGPIO_MODE_ALTERNATE_PP, stm32::AGPIO_SPEED_HALF );
		AFIO->MAPR |= AFIO_MAPR_TIM3_REMAP_FULLREMAP;
		stm32::tim_timebase_init( TIM3, 1, TIM_CounterMode_Up, 256, 0, 0 );
		stm32::tim_oc_init( TIM3, stm32::tim_cc_chn1, TIM_OCMode_PWM1, 50,
				TIM_OutputState_Enable, 0, TIM_OCPolarity_High, TIM_OCPolarity_High, 0, 0);
		stm32::tim_arrp_reload_config(TIM3, true);
		stm32::tim_cmd(TIM3, true );
	}

	static constexpr uint16_t xcolor( uint8_t r, uint8_t g, uint8_t b )
	{
		return ((b>>3)<<(16-5)) | ((g>>2)<<(16-5-6) | (r>>3) );
	}
protected:
	virtual void main()
	{
		gdisp.power_ctl( gfx::drv::power_ctl_t::on );
		isix::tick_t tbeg = isix::isix_get_jiffies();
		gdisp.clear( xcolor(255,0,0 ) );
		dbprintf("TIME=%u", isix::isix_get_jiffies() - tbeg );
	}
private:
	static const unsigned STACK_SIZE = 2048;
	static const unsigned TASK_PRIO = 3;
	//ili_bus ibus;
	ili_gpio_bus gbus;
	gfx::drv::ili9341 gdisp;
};


/* ------------------------------------------------------------------ */

}	//namespace app end
/* ------------------------------------------------------------------ */
//App main entry point
int main()
{
	 dblog_init( stm32::usartsimple_putc, NULL, stm32::usartsimple_init,
	    		USART2,115200,true, CONFIG_PCLK1_HZ, CONFIG_PCLK2_HZ );
	dbprintf("TFT Tester. Good Morning");
	//The blinker class
	static app::ledblink led_blinker;
	//The ledkey class
	static app::tft_tester ft;
	//static app::mmc_host_tester ht;
	isix::isix_wait_ms(1000);
	//Start the isix scheduler
	isix::isix_start_scheduler();
}

/* ------------------------------------------------------------------ */

