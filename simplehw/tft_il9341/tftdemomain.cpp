#include <isix.h>
#include <stm32lib.h>
#include <foundation/dbglog.h>
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
#include <gfx/types.hpp>
#include <gfx/disp/gdi.hpp>
#include <gfx/gui/gui.hpp>

#include <string>
#include <regex>
/* ------------------------------------------------------------------ */
namespace testimg {
	extern const gfx::disp::cmem_bitmap_t isixlogo_png;
	extern const gfx::disp::cmem_bitmap_t bat_png;
}
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
		/* PWM Configuration
		 *
		 */
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
	virtual ~ili_gpio_bus() {}
	// Lock bus and set address
	virtual void set_ctlbits( int bit, bool val )
	{
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
			nop(); nop(); nop(); nop();
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
	virtual void fill( unsigned value, size_t nelms )
	{
		using namespace stm32;
		bus_dir( dir_t::out );
		for( size_t l=0; l<nelms; ++l )
		{
			//One write
			gpio_set_clr_mask( DATA_PORT ,value, DATA_MASK );
			gpio_clr( CTL_PORT, WR_PIN );	//WR down
			gpio_set( CTL_PORT, WR_PIN );	//WR up
			gpio_set_clr_mask( DATA_PORT ,value>>8, DATA_MASK );
			gpio_clr( CTL_PORT, WR_PIN );	//WR down
			gpio_set( CTL_PORT, WR_PIN );	//WR up
		}
	}
	/* Wait ms long delay */
	virtual void delay( unsigned tout )
	{
		isix::isix_wait_ms( tout );
	}
	/* Set PWM  */
	virtual void set_pwm( int percent )
	{

	}
private:
	dir_t m_dir { dir_t::in };
};
/* ------------------------------------------------------------------ */


class tft_tester: public isix::task_base
{
private:
	//A window calllback for select item
	bool window_callback( const gfx::gui::event &ev )
	{
		using namespace gfx::input;
		bool ret {};
		//if( ev.keyb )
		if( ev.keyb.stat == detail::keyboard_tag::status::DOWN )
		{
			auto win = static_cast<gfx::gui::window*>(ev.sender);
			if( ev.keyb.key == kbdcodes::os_arrow_left )
			{
				win->select_prev();
			}
			else if( ev.keyb.key == kbdcodes::os_arrow_right )
			{
				win->select_next();
			}
			ret = true;
		}
		return ret;//	Need redraw
	}
	//Buttons callback
	bool on_click( const gfx::gui::event &ev )
	{
		auto b = static_cast<gfx::gui::button*>(ev.sender);
		dbprintf("Button %p clicked with desc %s", ev.sender, b->caption().c_str());
		return false;
	}
	//On select item
	bool on_select_item( const gfx::gui::event &ev )
	{
		auto b = static_cast<gfx::gui::choice_menu*>(ev.sender);
		dbprintf("Button %p clicked with desc %i", ev.sender, b->selection());
		return false;
	}
public:
	//Constructor
	tft_tester()
		: task_base(STACK_SIZE,TASK_PRIO),
		  gdisp( gbus ), frame(gdisp)
	{
	}
	gfx::gui::frame& get_frame()
	{
		return frame;
	}
protected:
	virtual void main()
	{
		//gdi_test();
		windows_test();
	}
private:
	//Base buttons and windows demo
	void windows_test()
	{
		using namespace gfx::gui;
		using namespace gfx::drv;
		//Menu for choice
		static constexpr choice_menu::item menu1[] = {
				{ 1, "ala ma kota" },
				{ 2, "kot ma ale" },
				{ 3, "zenek ma VIPA" },
				{ 4, "linia 4" },
				{ 5, "linia 5" },
				{ 6, "linia 6" },
				{ 7, "linia 7" },
				{ 8, "linia 8" },
				{ 9, "linia 9" },
				{ 9, "linia 10" },
				{ 9, "linia 11" },
				choice_menu::end	//Termination
		};

		gdisp.power_ctl( power_ctl_t::on );
		window win( rectangle( 10, 10, 200, 300), frame, window::flags::border |window::flags::selectborder );
		button btn( rectangle(20, 20, 100 , 40), layout(), win );
		button btn1( rectangle(20, 65, 100 , 40), layout(), win );
		label lbl1( rectangle(20, 120, 50 , 12), layout(), win );
		icon  ico1( rectangle(80, 120, 25 , 25), layout(), win );

		choice_menu choice1( rectangle(20, 147, 178, 120), layout(), win, choice_menu::style::normal );
		btn.caption("ALA");
		btn.pushkey( gfx::input::kbdcodes::enter );
		btn1.pushkey( gfx::input::kbdcodes::enter );
		btn1.caption("ELA");
		lbl1.caption("LABEL");
		ico1.image( testimg::bat_png );
		choice1.items( menu1 );
		//Connect windows callback to the main window
		win.connect(std::bind(&tft_tester::window_callback,this,std::placeholders::_1));
		btn.connect(std::bind(&tft_tester::on_click,this,std::placeholders::_1));
		btn1.connect(std::bind(&tft_tester::on_click,this,std::placeholders::_1));
		choice1.connect(std::bind(&tft_tester::on_select_item,this,std::placeholders::_1));
		frame.execute();
	}
	void gdi_test()
	{
		static uint16_t img[16*16];
		for( int x=0;x<16;x++ )
		for( int y=0;y<16;y++ )
		{
			if( y == 8 || y==0 || x==0 ||y==15||x==15)
				img[x + 16*y ] = gfx::rgb( 255 ,0, 255 );
			else
				img[x + 16*y ] = gfx::rgb( 255,255,0 );
		}
		gdisp.power_ctl( gfx::drv::power_ctl_t::on );
		gfx::disp::gdi gdi(gdisp);
		isix::tick_t tbeg = isix::isix_get_jiffies();
		gdi.clear();
		dbprintf("TIME=%u", isix::isix_get_jiffies() - tbeg );
		gdisp.fill(50,150,30,30, gfx::rgb(127,0,255));
		gdisp.blit( 20, 40, 16, 16, 0, img );
		tbeg = isix::isix_get_jiffies();
		//gdisp.blit( 5, 10, gimp_image.width, gimp_image.height, 0, (uint16_t*)gimp_image.pixel_data);
		dbprintf("BLIT=%u", isix::isix_get_jiffies() - tbeg );
		dbprintf( "PIX1=%04X %04x", gdisp.get_pixel( 1, 1 ) ,  gfx::rgb(0,255,0)  );
		dbprintf( "PIX2=%04X %04x", gdisp.get_pixel(140,140 ), gfx::rgb(255,0,0) );
		//isix::isix_wait_ms( 2000 );
		//gdisp.vert_scroll( 0, 0, 240/4, 320 ,-320/2, gfx::rgb(255,255,255));
		gdi.set_fg_color( gfx::rgb(64,255,45) );
		constexpr char txt[] = "Ala testuje napis a ciekawe co bedzie jak wyjdzie";
		tbeg = isix::isix_get_jiffies();
		gdi.draw_text(1,2, txt);
		dbprintf("string_time=%u", isix::isix_get_jiffies() - tbeg );
		tbeg = isix::isix_get_jiffies();
		gdi.draw_line( 1, 100, 100, 100 );
		dbprintf("line_time=%u", isix::isix_get_jiffies() - tbeg );
		tbeg = isix::isix_get_jiffies();
		gdi.draw_circle( 150, 150, 30 );
		dbprintf("circle_time=%u", isix::isix_get_jiffies() - tbeg );
		tbeg = isix::isix_get_jiffies();
		gdi.draw_ellipse( 50, 150, 30, 40 );
		dbprintf("ellipse_time=%u", isix::isix_get_jiffies() - tbeg );
		dbprintf("text_width=%u", gdi.get_text_width(txt));
		tbeg = isix::isix_get_jiffies();
		gdi.set_fg_color( gfx::rgb(255,0,0) );
		gdi.set_bg_color( gfx::rgb(0,0,0) );
		//gdi.draw_image(40, 200, testimg::isixlogo_png );
		dbprintf("drawimage_time=%u", isix::isix_get_jiffies() - tbeg );
	}
private:
	static const unsigned STACK_SIZE = 2048;
	static const unsigned TASK_PRIO = 3;
	//ili_bus ibus;
	ili_gpio_bus gbus;
	gfx::drv::ili9341 gdisp;
	gfx::gui::frame frame;
};
/* ------------------------------------------------------------------ */
class gpio_keypad: public isix::task_base
{
	static constexpr unsigned _bv( unsigned x )
	{
		return 1<<x;
	}
	static constexpr auto j_port = GPIOE;
	static constexpr auto j_ok = 8;
	static constexpr auto j_up = 9;
	static constexpr auto j_down = 10;
	static constexpr auto j_right = 11;
	static constexpr auto j_left = 12;
	bool io( unsigned pin )
	{
		return stm32::gpio_get( j_port, pin );
	}
	using ks = gfx::input::detail::keyboard_tag::status;
	void report_key( char key , ks type )
	{
		const gfx::input::event_info ei
		{
			isix::isix_get_jiffies(),
			gfx::input::event_info::evtype::EV_KEY,
			{
				type,
				key,
				0
			}
		};
		m_frm.report_event( ei );
	}
public:
	gpio_keypad( gfx::gui::frame& frm )
		: task_base(STACK_SIZE,TASK_PRIO), m_frm(frm)
	{
		stm32::gpio_clock_enable(j_port, true);
		stm32::gpio_abstract_config_ext( j_port,
				_bv(j_ok)|_bv(j_up)|_bv(j_down)|_bv(j_left)|_bv(j_right),
				stm32::AGPIO_MODE_INPUT_PULLUP, stm32::AGPIO_SPEED_FULL
		);
	}
	virtual void main()
	{
		using keys = gfx::input::kbdcodes;
		bool pok{}, pl{}, pr{}, pu{}, pd{};
		for(;;)
		{
			//ok
			const bool ok = io(j_ok);
			if( !ok && pok ) report_key( keys::enter, ks::DOWN );
			else if( ok && !pok ) report_key( keys::enter, ks::UP );
			pok = ok;
			//left
			const bool l = io(j_left);
			if( !l && pl ) report_key( keys::os_arrow_left, ks::DOWN );
			if( l && !pl ) report_key( keys::os_arrow_left, ks::UP );
			pl = l;
			//right
			const bool r = io(j_right);
			if( !r && pr ) report_key( keys::os_arrow_right, ks::DOWN );
			if( r && !pr ) report_key( keys::os_arrow_right, ks::UP );
			pr = r;
			//right
			const bool u = io(j_up);
			if( !u && pu ) report_key( keys::os_arrow_up, ks::DOWN );
			if( u && !pu ) report_key( keys::os_arrow_up, ks::UP );
			pu = u;
			//down
			const bool d = io(j_down);
			if( !d && pd ) report_key( keys::os_arrow_down, ks::DOWN );
			if( d && !pd ) report_key( keys::os_arrow_down, ks::UP );
			pd = d;
			isix::isix_wait_ms(20);
		}
	}
private:
	static const unsigned STACK_SIZE = 384;
	static const unsigned TASK_PRIO = 3;
	gfx::gui::frame& m_frm;
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

	static app::gpio_keypad kp( ft.get_frame() );

	//static app::mmc_host_tester ht;
	isix::isix_wait_ms(1000);
	//Start the isix scheduler
	isix::isix_start_scheduler();
}

/* ------------------------------------------------------------------ */

