/*
 * =====================================================================================
 *
 *       Filename:  keypad.cpp
 *
 *    Description:  Keypad driver for graphics library
 *
 *        Version:  1.0
 *        Created:  27.08.2016 10:51:47
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Lucjan Bryndza (LB), lbryndza.p(at)boff(dot)pl
 *   Organization:  BoFF
 *
 * =====================================================================================
 */

#include <board/keypad.hpp>
#include <stm32_ll_gpio.h>


namespace dev {

// Port configuration
namespace {
	static const auto key_port = GPIOF;
	static const auto key_portok = GPIOB;
	static constexpr auto key_left = 12;
	static constexpr auto key_right = 13;
	static constexpr auto key_up = 14;
	static constexpr auto key_down = 15;
	static constexpr auto key_ok = 2;

	bool io( unsigned pin ) {
		return LL_GPIO_IsInputPinSet(key_port, 1U<<pin);
	}
	bool io() {
		return LL_GPIO_IsInputPinSet(key_portok, 1U<<key_ok);
	}

}

// Constructor
keypad::keypad( gfx::gui::frame& frm )
	: m_thr( isix::thread_create_and_run( c_thr_stk, isix::get_min_priority(), 0,
		std::bind( &keypad::thread, this ) ) )
	, m_frm( frm )
{
	LL_GPIO_InitTypeDef port_cnf {
		.Pin = _bv(key_left)|_bv(key_right)|_bv(key_up)|_bv(key_down),
		.Mode = LL_GPIO_MODE_INPUT,
		.Speed = LL_GPIO_SPEED_FREQ_LOW,
		.OutputType = LL_GPIO_OUTPUT_PUSHPULL,
		.Pull = LL_GPIO_PULL_UP,
		.Alternate = 0
	};
	LL_GPIO_Init(key_port, &port_cnf);
	port_cnf.Pin = _bv(key_ok);
	LL_GPIO_Init(key_portok, &port_cnf);
}



//Report key to gui engine
void keypad::report_key( char key , ks type )
{
	const gfx::input::event_info ei {
		isix::get_jiffies(),
		gfx::input::event_info::evtype::EV_KEY,
		nullptr, {
			type, key, 0, 0
		}
	};
	m_frm.report_event( ei );
}


// Thread configuration data
void keypad::thread() noexcept
{
		using keys = gfx::input::kbdcodes;
		bool pok{}, pl{}, pr{}, pu{}, pd{};
		for(;;)
		{
			//ok
			const bool ok = io();
			if( !ok && pok ) report_key( keys::enter, ks::DOWN );
			else if( ok && !pok ) report_key( keys::enter, ks::UP );
			pok = ok;
			//left
			const bool l = io(key_left);
			if( !l && pl ) report_key( keys::os_arrow_left, ks::DOWN );
			if( l && !pl ) report_key( keys::os_arrow_left, ks::UP );
			pl = l;
			//right
			const bool r = io(key_right);
			if( !r && pr ) report_key( keys::os_arrow_right, ks::DOWN );
			if( r && !pr ) report_key( keys::os_arrow_right, ks::UP );
			pr = r;
			//right
			const bool u = io(key_up);
			if( !u && pu ) report_key( keys::os_arrow_up, ks::DOWN );
			if( u && !pu ) report_key( keys::os_arrow_up, ks::UP );
			pu = u;
			//down
			const bool d = io(key_down);
			if( !d && pd ) report_key( keys::os_arrow_down, ks::DOWN );
			if( d && !pd ) report_key( keys::os_arrow_down, ks::UP );
			pd = d;
			isix::wait_ms(c_keypad_delay);
		}
}


}

