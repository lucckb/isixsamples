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


/* ------------------------------------------------------------------ */
#define 	ILI9341_CMD_BACKLIGHT_CONTROL_1   0xB8
#define 	ILI9341_CMD_BACKLIGHT_CONTROL_2   0xB9
#define 	ILI9341_CMD_BACKLIGHT_CONTROL_3   0xBA
#define 	ILI9341_CMD_BACKLIGHT_CONTROL_4   0xBB
#define 	ILI9341_CMD_BACKLIGHT_CONTROL_5   0xBC
#define 	ILI9341_CMD_BACKLIGHT_CONTROL_6   0xBD
#define 	ILI9341_CMD_BACKLIGHT_CONTROL_7   0xBE
#define 	ILI9341_CMD_BACKLIGHT_CONTROL_8   0xBF
#define 	ILI9341_CMD_BLANKING_PORCH_CONTROL   0xB5
#define 	ILI9341_CMD_COLMOD_PIXEL_FORMAT_SET   0x3A
#define 	ILI9341_CMD_COLOR_SET   0x2D
#define 	ILI9341_CMD_COLUMN_ADDRESS_SET   0x2A
#define 	ILI9341_CMD_DIGITAL_GAMMA_CONTROL_1   0xE2
#define 	ILI9341_CMD_DIGITAL_GAMMA_CONTROL_2   0xE3
#define 	ILI9341_CMD_DISP_INVERSION_OFF   0x20
#define 	ILI9341_CMD_DISP_INVERSION_ON   0x21
#define 	ILI9341_CMD_DISPLAY_FUNCTION_CONTROL   0xB6
#define 	ILI9341_CMD_DISPLAY_INVERSION_CONTROL   0xB4
#define 	ILI9341_CMD_DISPLAY_OFF   0x28
#define 	ILI9341_CMD_DISPLAY_ON   0x29
#define 	ILI9341_CMD_DRIVER_TIMING_CONTROL_A   0xE8
#define 	ILI9341_CMD_DRIVER_TIMING_CONTROL_B   0xEA
#define 	ILI9341_CMD_ENABLE_3_GAMMA_CONTROL   0xF2
#define 	ILI9341_CMD_ENTER_SLEEP_MODE   0x10
#define 	ILI9341_CMD_ENTRY_MODE_SET   0xB7
#define 	ILI9341_CMD_FRAME_RATE_CONTROL_IDLE_8COLOR   0xB2
#define 	ILI9341_CMD_FRAME_RATE_CONTROL_NORMAL   0xB1
#define 	ILI9341_CMD_FRAME_RATE_CONTROL_PARTIAL   0xB3
#define 	ILI9341_CMD_GAMMA_SET   0x26
#define 	ILI9341_CMD_GET_SCANLINE   0x45
#define 	ILI9341_CMD_IDLE_MODE_OFF   0x38
#define 	ILI9341_CMD_IDLE_MODE_ON   0x39
#define 	ILI9341_CMD_INTERFACE_CONTROL   0xF6
#define 	ILI9341_CMD_MEMORY_ACCESS_CONTROL   0x36
#define 	ILI9341_CMD_MEMORY_READ   0x2E
#define 	ILI9341_CMD_MEMORY_WRITE   0x2C
#define 	ILI9341_CMD_NEGATIVE_GAMMA_CORRECTION   0xE1
#define 	ILI9341_CMD_NOP   0x00
#define 	ILI9341_CMD_NORMAL_DISP_MODE_ON   0x13
#define 	ILI9341_CMD_NVMEM_PROTECTION_KEY   0xD1
#define 	ILI9341_CMD_NVMEM_STATUS_READ   0xD2
#define 	ILI9341_CMD_NVMEM_WRITE   0xD0
#define 	ILI9341_CMD_PAGE_ADDRESS_SET   0x2B
#define 	ILI9341_CMD_PARTIAL_AREA   0x30
#define 	ILI9341_CMD_PARTIAL_MODE_ON   0x12
#define 	ILI9341_CMD_POSITIVE_GAMMA_CORRECTION   0xE0
#define 	ILI9341_CMD_POWER_CONTROL_1   0xC0
#define 	ILI9341_CMD_POWER_CONTROL_2   0xC1
#define 	ILI9341_CMD_POWER_CONTROL_A   0xCD
#define 	ILI9341_CMD_POWER_CONTROL_B   0xCF
#define 	ILI9341_CMD_POWER_ON_SEQ_CONTROL   0xCB
#define 	ILI9341_CMD_PUMP_RATIO_CONTROL   0xF7
#define 	ILI9341_CMD_READ_CONTENT_ADAPT_BRIGHTNESS   0x56
#define 	ILI9341_CMD_READ_CTRL_DISPLAY   0x54
#define 	ILI9341_CMD_READ_DISP_ID   0x04
#define 	ILI9341_CMD_READ_DISP_IMAGE_FORMAT   0x0D
#define 	ILI9341_CMD_READ_DISP_MADCTRL   0x0B
#define 	ILI9341_CMD_READ_DISP_PIXEL_FORMAT   0x0C
#define 	ILI9341_CMD_READ_DISP_SELF_DIAGNOSTIC   0x0F
#define 	ILI9341_CMD_READ_DISP_SIGNAL_MODE   0x0E
#define 	ILI9341_CMD_READ_DISP_STATUS   0x09
#define 	ILI9341_CMD_READ_DISPLAY_BRIGHTNESS   0x52
#define 	ILI9341_CMD_READ_ID1   0xDA
#define 	ILI9341_CMD_READ_ID2   0xDB
#define 	ILI9341_CMD_READ_ID3   0xDC
#define 	ILI9341_CMD_READ_ID4   0xD3
#define 	ILI9341_CMD_READ_MEMORY_CONTINUE   0x3E
#define 	ILI9341_CMD_READ_MIN_CAB_LEVEL   0x5F
#define 	ILI9341_CMD_RGB_SIGNAL_CONTROL   0xB0
#define 	ILI9341_CMD_SET_TEAR_SCANLINE   0x44
#define 	ILI9341_CMD_SLEEP_OUT   0x11
#define 	ILI9341_CMD_SOFTWARE_RESET   0x01
#define 	ILI9341_CMD_TEARING_EFFECT_LINE_OFF   0x34
#define 	ILI9341_CMD_TEARING_EFFECT_LINE_ON   0x35
#define 	ILI9341_CMD_VCOM_CONTROL_1   0xC5
#define 	ILI9341_CMD_VCOM_CONTROL_2   0xC7
#define 	ILI9341_CMD_VERT_SCROLL_DEFINITION   0x33
#define 	ILI9341_CMD_VERT_SCROLL_START_ADDRESS   0x37
#define 	ILI9341_CMD_WRITE_CONTENT_ADAPT_BRIGHTNESS   0x55
#define 	ILI9341_CMD_WRITE_CTRL_DISPLAY   0x53
#define 	ILI9341_CMD_WRITE_DISPLAY_BRIGHTNESS   0x51
#define 	ILI9341_CMD_WRITE_MEMORY_CONTINUE   0x3C
#define 	ILI9341_CMD_WRITE_MIN_CAB_LEVEL   0x5E

#define 	ILI9341_FLIP_X   1
#define 	ILI9341_FLIP_Y   2
#define 	ILI9341_SWITCH_XY   4
#define 	ILI9341_DEFAULT_HEIGHT   320
#define 	ILI9341_DEFAULT_WIDTH   240
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


class ili_bus
{
	ili_bus(const ili_bus&) = delete;
	ili_bus& operator=(const ili_bus&) = delete;
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
	void bus_dir( bool wr )
	{
		using namespace stm32;
		if( wr && !m_wr_dir )
		{
			gpio_abstract_config_ext( DATA_PORT, DATA_MASK, AGPIO_MODE_OUTPUT_PP, AGPIO_SPEED_FULL );
		}
		else if( !wr && m_wr_dir )
		{
			gpio_abstract_config_ext( DATA_PORT, DATA_MASK, AGPIO_MODE_INPUT_PULLUP, AGPIO_SPEED_FULL );
		}
		m_wr_dir = wr;
	}
public:
	/* Constructor */
	ili_bus()
	{
		using namespace stm32;
		gpio_clock_enable( DATA_PORT, true );
		gpio_clock_enable( CTL_PORT, true );
		gpio_abstract_config_ext( DATA_PORT, DATA_MASK, AGPIO_MODE_INPUT_PULLUP, AGPIO_SPEED_FULL );
		gpio_abstract_config_ext( CTL_PORT,
			bv(CS_PIN)|bv(RS_PIN)|bv(WR_PIN)|bv(RD_PIN)|bv(RST_PIN),
			AGPIO_MODE_OUTPUT_PP, AGPIO_SPEED_FULL
		);
		gpio_set( CTL_PORT, CS_PIN );
		gpio_set( CTL_PORT, WR_PIN );
		gpio_set( CTL_PORT, RD_PIN );

		gpio_set( CTL_PORT, RST_PIN );
		isix::isix_wait_ms( 10 );
		gpio_clr( CTL_PORT, RST_PIN );
		isix::isix_wait_ms( 10 );
		gpio_set( CTL_PORT, RST_PIN );
		isix::isix_wait_ms( 150 );
	}
	void sel_mode( bool command_mode )
	{
		using namespace stm32;
		if( command_mode )  gpio_clr( CTL_PORT, RS_PIN );
		else				gpio_set( CTL_PORT, RS_PIN );
	}
	/* Write command */
	void write( uint8_t data )
	{
		using namespace stm32;
		bus_dir( 1 );
		gpio_set_clr_mask( DATA_PORT ,data, DATA_MASK );
		gpio_clr( CTL_PORT, WR_PIN );	//WR down
		gpio_set( CTL_PORT, WR_PIN );	//WR up
	}
	/* Read command */
	uint8_t read()
	{
		using namespace stm32;
		bus_dir( 0 );
		gpio_clr( CTL_PORT, RD_PIN );
		nop();
		const uint8_t v = gpio_get_mask( DATA_PORT, DATA_MASK );
		gpio_set( CTL_PORT, RD_PIN );
		return v;
	}
	void act( bool on )
	{
		using namespace stm32;
		if( on )
			gpio_clr( CTL_PORT, CS_PIN );	//CS down
		else
			gpio_set( CTL_PORT, CS_PIN );	// CS up
	}
	void command( uint8_t cmd )
	{
		sel_mode( 1 );
		act( 1 );
		write( cmd );
		sel_mode( 0 );
	}
private:
	bool m_wr_dir { false };
};



class tft_tester: public isix::task_base
{
public:
	//Constructor
	tft_tester()
		: task_base(STACK_SIZE,TASK_PRIO)
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

	void exit_standby()
	{
		ibus.command(ILI9341_CMD_SLEEP_OUT );
	    ibus.act( 0 );
	    isix::isix_wait_ms(150);
	    ibus.command(ILI9341_CMD_DISPLAY_ON);
	    ibus.act( 0 );
	}

	void set_orientation(uint8_t flags)
	{
	    uint8_t madctl = 0x48;

	    if (flags & ILI9341_FLIP_X) {
	        madctl &= ~(1 << 6);
	    }

	    if (flags & ILI9341_FLIP_Y) {
	        madctl |= 1 << 7;
	    }

	    if (flags & ILI9341_SWITCH_XY) {
	        madctl |= 1 << 5;
	    }

	    ibus.command(ILI9341_CMD_MEMORY_ACCESS_CONTROL);
	    ibus.write(madctl);
	    ibus.act( 0 );
	}

	void set_limits(int start_x, int start_y, int end_x = 0, int end_y = 0 )
	{
		ibus.command(ILI9341_CMD_COLUMN_ADDRESS_SET);
		ibus.write(start_x >> 8);
		ibus.write(start_x & 0xFF);
	    if (end_x>0)
	    {
	    	ibus.write(end_x >> 8);
	    	ibus.write(end_x & 0xFF);
	    }
	    ibus.act( 0 );
	    ibus.command(ILI9341_CMD_PAGE_ADDRESS_SET);
	    ibus.write(start_y >> 8);
	    ibus.write(start_y & 0xFF);
	    if (end_y>0)
	    {
	    	ibus.write(end_y >> 8);
	    	ibus.write(end_y & 0xFF);
	    }
	    ibus.act( 0 );
	}

	void init_tft( )
	{
		ibus.command(ILI9341_CMD_POWER_CONTROL_A);
		ibus.write(0x39);
		ibus.write(0x2C);
		ibus.write(0x00);
		ibus.write(0x34);
		ibus.write(0x02);
		ibus.act( 0 );

		ibus.command(ILI9341_CMD_POWER_CONTROL_B);
		ibus.write(0x00);
		ibus.write(0xAA);
		ibus.write(0XB0);
		ibus.act( 0 );

		ibus.command(ILI9341_CMD_PUMP_RATIO_CONTROL);
		ibus.write(0x30);
		ibus.act( 0 );

		ibus.command(ILI9341_CMD_POWER_CONTROL_1);
		ibus.write(0x25);
		ibus.act( 0 );

		ibus.command(ILI9341_CMD_POWER_CONTROL_2);
		ibus.write(0x11);
		ibus.act( 0 );

		ibus.command(ILI9341_CMD_VCOM_CONTROL_1);
		ibus.write(0x5C);
		ibus.write(0x4C);
		ibus.act( 0 );

		ibus.command(ILI9341_CMD_VCOM_CONTROL_2);
		ibus.write(0x94);
		ibus.act( 0 );

		ibus.command(ILI9341_CMD_DRIVER_TIMING_CONTROL_A);
		ibus.write(0x85);
		ibus.write(0x01);
		ibus.write(0x78);
		ibus.act( 0 );

		ibus.command(ILI9341_CMD_DRIVER_TIMING_CONTROL_B);
		ibus.write(0x00);
		ibus.write(0x00);
		ibus.act( 0 );

		ibus.command(ILI9341_CMD_COLMOD_PIXEL_FORMAT_SET);
		ibus.write(0x05);
		ibus.act( 0 );

		set_orientation(0);
		set_limits(0, 0, ILI9341_DEFAULT_WIDTH, ILI9341_DEFAULT_HEIGHT);
	}

	void tft_init2()
	{
		ibus.command( 0xCF );
		ibus.write(0x00);
		ibus.write(0x81);
		ibus.write(0x00);
		ibus.write(0x30);
		ibus.act( 0 );

		ibus.command( 0xED );
		ibus.write(0x64);
		ibus.write(0x03);
		ibus.write(0x12);
		ibus.write(0x81);
		ibus.act( 0 );

		ibus.command( 0xE8 );
		ibus.write(0x85);
		ibus.write(0x00);
		ibus.write(0x00);
		ibus.write(0x79);
		ibus.act( 0 );

		ibus.command( 0xC8 );
		ibus.write(0x39);
		ibus.write(0x2c);
		ibus.write(0x00);
		ibus.write(0x34);
		ibus.write(0x00);
		ibus.write(0x02);
		ibus.act( 0 );

		ibus.command( 0xF7 );
		ibus.write(0x20);
		ibus.act( 0 );

		ibus.command( 0xEA );
		ibus.write(0x00);
		ibus.write(0x00);
		ibus.act( 0 );

		ibus.command( 0xB1 );
		ibus.write(0x00);
		ibus.write(0x1b);
		ibus.act( 0 );

		ibus.command( 0xC0 );
		ibus.write(0x2e);
		ibus.act( 0 );

		ibus.command( 0xC1 );
		ibus.write(0x12);
		ibus.act( 0 );

		ibus.command( 0xC5 );
		ibus.write(0x50);
		ibus.write(0x19);
		ibus.act( 0 );

		ibus.command( 0xC7 );
		ibus.write(0x90);
		ibus.act( 0 );

		ibus.command( 0x36 );
		ibus.write(0xA8);
		ibus.act( 0 );

		ibus.command( 0xB6 );
		ibus.write(0x0A);
		ibus.write(0xA2);
		ibus.act( 0 );

		ibus.command( 0xF2 );
		ibus.write(0x00);
		ibus.act( 0 );

		ibus.command( 0x26 );
		ibus.write(0x01);
		ibus.act( 0 );

		ibus.command( 0xE0 );
		ibus.write(0x0f);
		ibus.write(0x27);
		ibus.write(0x23);
		ibus.write(0x0b);
		ibus.write(0x0f);
		ibus.write(0x05);
		ibus.write(0x54);
		ibus.write(0x74);
		ibus.write(0x45);
		ibus.write(0x0A);
		ibus.write(0x17);
		ibus.write(0x0A);
		ibus.write(0x1c);
		ibus.write(0x0e);
		ibus.write(0x00);
		ibus.write(0x08);
		ibus.act( 0 );

		ibus.command( 0xE1 );
		ibus.write(0x08);
		ibus.write(0x1A);
		ibus.write(0x1E);
		ibus.write(0x03);
		ibus.write(0x0f);
		ibus.write(0x05);
		ibus.write(0x2e);
		ibus.write(0x25);
		ibus.write(0x3b);
		ibus.write(0x01);
		ibus.write(0x06);
		ibus.write(0x05);
		ibus.write(0x25);
		ibus.write(0x33);
		ibus.write(0x00);
		ibus.write(0x0f);
		ibus.act( 0 );

		ibus.command( 0x3a );
		ibus.write(0x55);
		ibus.act( 0 );

		ibus.command( 0x36 );
		ibus.write(0x00);
		ibus.act( 0 );

		ibus.command( 0xF6 );
		ibus.write(0x01);
		ibus.write(0x30);
		ibus.act( 0 );

		ibus.command( 0x29 );
		ibus.act( 0 );

		ibus.command( 0x11 );
		ibus.act( 0 );
		isix::isix_wait_ms(500);

		ibus.command( 0x2A );
		ibus.write(0x00);
		ibus.write(0x00);
		ibus.write( ILI9341_DEFAULT_WIDTH >> 8);
		ibus.write( ILI9341_DEFAULT_WIDTH );
		ibus.act( 0 );

		ibus.command( 0x2B );
		ibus.write(0x00);
		ibus.write(0x00);
		ibus.write( ILI9341_DEFAULT_HEIGHT >> 8);
		ibus.write( ILI9341_DEFAULT_HEIGHT );
		ibus.act( 0 );


		ibus.command( 0xF6 );
		ibus.write(0x01);
		ibus.write(0x30);
		ibus.act( 0 );

	}
	static constexpr uint16_t xcolor( uint8_t r, uint8_t g, uint8_t b )
	{
		return ((b>>3)<<(16-5)) | ((g>>2)<<(16-5-6) | (r>>3) );
	}
protected:
	//Main function
	virtual void main()
	{
		//exit_standby();
		//init_tft();
		tft_init2();

		while(1)
		{
			isix::tick_t tbeg = isix::isix_get_jiffies();
			ibus.command( ILI9341_CMD_MEMORY_WRITE );
			uint16_t color = xcolor( 0xff, 0xff , 0 );
			for(auto i=0;i<320*240;i++ )
			{
				ibus.write( color >> 8 );
				ibus.write( color );
				//isix::isix_wait_ms( 1 );
			}
			ibus.act( 0 );
			dbprintf("TIME=%u", isix::isix_get_jiffies() - tbeg );

			isix::isix_wait_ms( 2000 );

			ibus.command( ILI9341_CMD_MEMORY_WRITE );
			color = xcolor( 0xff, 0 , 0 );
			for(auto i=0;i<320*240;i++ )
			{
				ibus.write( color >> 8 );
				ibus.write( color );
			}
			ibus.act( 0 );

			isix::isix_wait_ms( 2000 );

			isix::isix_wait_ms( 60 );
			ibus.command( 0x0C );
			ibus.read();
			dbprintf("%02x", ibus.read());
			ibus.act( 0 );
		}

	}
private:
	static const unsigned STACK_SIZE = 2048;
	static const unsigned TASK_PRIO = 3;
	ili_bus ibus;
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

