#include <isix.h>
#include <foundation/dbglog.h>
#include <foundation/tiny_printf.h>
#include <usart_simple.h>
#include <config/conf.h>
#include <stm32system.h>
#include <cctype>
#include <cstring>
#include <functional>
#include <stm32gpio.h>
#include <string>
#include <usb/host/internal.h>
#include <usb/host/controller.h>
#include <usb/drivers/hostdev/hid_keyboard.h>
#include <usb/drivers/hostdev/hid_joystick.h>
#include <algorithm>
#include "blinker.hpp"
/* ------------------------------------------------------------------ */
//Anonymous namespace for special keyboard handler
namespace {
	void  kbd_connected( const usbh_keyb_hid_context_t* id ) {
		dbprintf("Keyb ID %p connected", id );
	}
	void  kbd_disconnected( const usbh_keyb_hid_context_t* id ) {
		dbprintf("Keyb ID %p disconnected", id );
	}
	void kbd_report( const usbh_keyb_hid_context_t* , const usbh_keyb_hid_event_t* evt ) {
		if( evt->key ) {
			fnd::tiny_printf("%c", evt->key );
		} else {
			//dbprintf("S %i %02x", evt->scan_code, evt->scan_code );
		}
	}

	void kbd_desc( const usbh_keyb_hid_context_t* id, usbh_driver_desc_type desc, const char *str ) {
		dbprintf("ID %p Desc %i str: %s", id, desc, str );
	}
	constexpr usbh_hid_kbd_ops kbd_fops = {
		kbd_connected,
		kbd_disconnected,
		kbd_report,
		kbd_desc
	};
}
/* ------------------------------------------------------------------ */ 
namespace {
	void joy_connected( const usbh_hid_joy_context_t* id ) {
		dbprintf("Joy ID %p, connected, " , id);
	}
	void joy_disconnected( const usbh_hid_joy_context_t* id ) {
		dbprintf("Joy ID %p, disconnected, " , id );
	}
	void joy_desc( const usbh_hid_joy_context_t* id, usbh_driver_desc_type desc, const char *str ) {
		dbprintf("Joy ID %p Desc %i str: %s", id, desc, str );
	}
	void joy_report( const usbh_hid_joy_context_t *id, const usbh_joy_hid_event_t* evt ) {
		dbprintf("Joy ID %p events", id );
		if( evt->has.X ) {
			dbprintf("X_pos=%u", evt->X );
		}
		if( evt->has.Y ) {
			dbprintf("Y_pos=%u", evt->Y );
		}
		if( evt->has.Z ) {
			dbprintf("Z_pos=%u", evt->Z );
		}
		if( evt->has.rX ) {
			dbprintf("rX_pos=%u", evt->rX );
		}
		if( evt->has.rY) {
			dbprintf("rY_pos=%u", evt->rY );
		}
		if( evt->has.rZ ) {
			dbprintf("rZ_pos=%u", evt->rZ );
		}
		if( evt->has.hat ) {
			dbprintf("hat_pos=%u", evt->hat );
		}
		if( evt->has.slider ) {
			dbprintf("slider_pos=%u", evt->slider );
		}
		if( evt->n_buttons > 0 ) {
			dbprintf("got %u buttons val %x", evt->n_buttons, evt->buttons );
		}
	}
	constexpr usbh_hid_joystick_ops joy_ops = {
		joy_connected,
		joy_disconnected,
		joy_report,
		joy_desc,
	};
}
/* ------------------------------------------------------------------ */
//App main entry point
int main()
{
	dblog_init( stm32::usartsimple_putc, NULL, stm32::usartsimple_init,
	    		USART2,115200, true, CONFIG_PCLK1_HZ, CONFIG_PCLK2_HZ );

	static app::ledblink blink;
	dbprintf("USBHost app started OK");
	// test
	usbh_controller_init(USB_PHY_A, 2);
	usbh_controller_attach_driver( usbh_hid_keyboard_init(&kbd_fops) );
	usbh_controller_attach_driver( usbh_hid_joystick_init(&joy_ops) );
	dbprintf("Joystick and KBD initialized and completed");
	//Start the isix scheduler
	isix::start_scheduler();

	dbprintf("Scheduler exit");
}

/* ------------------------------------------------------------------ */

