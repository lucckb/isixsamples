/**
 * =====================================================================================
 * 	File: tft_livedemo.hpp
 * 	Created Date: Sunday, August 25th 2019, 8:35:41 pm
 * 	Author: Lucjan Bryndza
 * 	Copyright (c) 2019 BoFF
 * 
 * 	GPL v2/3
 * =====================================================================================
 */
#include <gfx/types.hpp>
#include <gfx/disp/gdi.hpp>
#include <gfx/gui/gui.hpp>
#include <gfx/drivers/disp/dsi_fb.hpp>
#include <periph/drivers/display/rgb/fbdev.hpp>
#include <periph/drivers/display/rgb/otm8009a.hpp>
#include <periph/drivers/display/bus/dsi.hpp>
#include <isix/thread.hpp>
#include <gfx/drivers/input/ft6x06_touch.hpp>
#include <periph/drivers/i2c/i2c_master.hpp>

namespace app {

class tft_livedemo
{
    static const unsigned STACK_SIZE = 2048;
	static const unsigned TASK_PRIO = 3;
public:
    //Constructor
    tft_livedemo(tft_livedemo&)=delete;
    tft_livedemo& operator=(tft_livedemo&)=delete;
    tft_livedemo();
    //Start the first task
    void start() noexcept;
private:
    //! Main thread
    void thread() noexcept;
    //! Lib test
    void windows_test() noexcept;
    //! Base GDI test
    void gdi_test() noexcept;
    //A window calllback for select item
	bool window_callback( const gfx::gui::event &ev );
	//Buttons callback
	bool on_click( const gfx::gui::event &ev );
	//On select item
	bool on_select_item( const gfx::gui::event &ev );
    //On seek change
	bool on_seek_change( const gfx::gui::event &ev );
    //! Setup i2c bus
    int setup_i2c_bus() noexcept;
private:
    periph::display::bus::dsi m_dsi { "dsi" };
    periph::display::fbdev m_fb { "ltdc" };
    periph::display::otm8009a displl {m_dsi, "display"};
    gfx::drv::dsi_fb m_disp { m_fb,displl };
    gfx::gui::frame frame { m_disp };
	gfx::gui::editbox* m_edit {};
	bool m_edit_mode = false;
    periph::drivers::i2c_master m_i2c { "i2c1" };
    gfx::drv::ft6x06_touch m_touch { "display", m_i2c, frame };
    isix::thread m_thr;
};

}
