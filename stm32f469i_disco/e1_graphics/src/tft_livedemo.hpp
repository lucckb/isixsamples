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

namespace app {

class tft_livedemo
{
public:
    //Constructor
    tft_livedemo(tft_livedemo&)=delete;
    tft_livedemo& operator=(tft_livedemo&)=delete;
    tft_livedemo();
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
private:
    periph::display::bus::dsi m_dsi { "dsi" };
    periph::display::fbdev m_fb { "ltdc" };
    periph::display::otm8009a displl {m_dsi, "display"};
    gfx::drv::dsi_fb m_disp { m_fb,displl };
    gfx::gui::frame frame;
	gfx::gui::editbox* m_edit {};
	bool m_edit_mode = false;
};

}
