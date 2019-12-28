/**
 * =====================================================================================
 * 	File: tft_livedemo.cpp
 * 	Created Date: Sunday, August 25th 2019, 11:56:46 pm
 * 	Author: Lucjan Bryndza
 * 	Copyright (c) 2019 BoFF
 * 
 * 	GPL v2/3
 * =====================================================================================
 */
#include "tft_livedemo.hpp"
#include <foundation/sys/dbglog.h>
#include <functional>
#include <isix.h>

namespace app {


// Main constructor
tft_livedemo::tft_livedemo()
    : m_thr(isix::thread_create( std::bind(&tft_livedemo::thread,std::ref(*this))))
{
}

//! Start the thread
void tft_livedemo::start() noexcept
{
    m_thr.start_thread(STACK_SIZE,TASK_PRIO);
}

//! Setup i2c bus
int tft_livedemo::setup_i2c_bus() noexcept
{
    int ret {};
    do {
        //Open with timeout
        ret = m_i2c.open(1000);
        if(ret) {
            dbg_err("Unable to open i2cdev %i", ret);
            break;
        }
        ret = m_i2c.set_option(periph::option::speed(350'000));
        if(ret) {
            dbg_err("Unbable to set speed option");
            break;
        }
    } while(0);
    return ret;
}

//A window calllback for select item
bool tft_livedemo::window_callback(const gfx::gui::event &ev)
{
    using namespace gfx::input;
    using namespace gfx::gui;
    bool ret{};
    auto win = static_cast<gfx::gui::window *>(ev.sender);
    if (ev.type==event::evtype::EV_KEY && ev.keyb.stat == gfx::input::detail::keyboard_tag::status::DOWN)
    {
        if (ev.keyb.key == kbdcodes::os_arrow_left && !m_edit_mode)
        {
            win->select_prev();
            ret = true;
        }
        else if (ev.keyb.key == kbdcodes::os_arrow_right && !m_edit_mode)
        {
            win->select_next();
            ret = true;
        }
        if (ev.keyb.key == kbdcodes::enter && m_edit == win->current_widget())
        {
            m_edit_mode = !m_edit_mode;
            m_edit->readonly(!m_edit_mode);
            dbg_info("Toggle edit mode %i", m_edit_mode);
        }
    }
    //Handle touch screen by selection
    if(ev.type==event::evtype::EV_TOUCH && ev.touch.eventid==touchevents::press_down) {
        win->select({ev.touch.x, ev.touch.y});
    }
    return ret; //	Need redraw
}

//Buttons callback
bool tft_livedemo::on_click( const gfx::gui::event &ev )
{
    auto b = static_cast<gfx::gui::button*>(ev.sender);
    static_cast<void>(b);
    dbg_info("Button %p clicked with desc %s", ev.sender, b->caption().c_str());
    return false;
}
//On select item
bool tft_livedemo::on_select_item( const gfx::gui::event &ev )
{
    auto b = static_cast<gfx::gui::choice_menu*>(ev.sender);
    static_cast<void>(b);
    dbg_info("Sel %p clicked with item %i", ev.sender, b->selection());
    return false;
}
bool tft_livedemo::on_seek_change( const gfx::gui::event &ev )
{
    auto b = static_cast<gfx::gui::seekbar*>(ev.sender);
    static_cast<void>(b);
    dbg_info("Seek event pos %i", b->value());
    return false;
}

void tft_livedemo::thread() noexcept
{
    if(setup_i2c_bus()) {
        return;
    }
    //Start touch screen only when init is ok
    m_touch.start();
    // Start the demo
    windows_test();
    //gdi_test();
}
//Base buttons and windows demo
void tft_livedemo::windows_test()
{
    using namespace gfx::gui;
    using namespace gfx::drv;
    //Menu for choice
    static constexpr choice_menu::item menu1[] = {
            { 1, "linia 1" },
            { 2, "linia 2" },
            { 3, "linia 3" },
            { 4, "linia 4" },
            { 5, "linia 5" },
            { 6, "linia 6" },
            { 7, "linia 7" },
            { 8, "linia 8" },
            { 9, "linia 9" },
            { 10, "linia 10" },
            { 11, "linia 11" },
            { 12, "linia 12" },
            { 13, "linia 13" },
            { 14, "linia 14" },
            { 15, "linia 15" },
            { 16, "linia 16" },
            { 17, "linia 17" },
            { 18, "linia 18" },
            { 19, "linia 19" },
            { 20, "linia 20" },
            choice_menu::end	//Termination
    };

    m_disp.power_ctl( power_ctl_t::on );
    window win( rectangle( 10, 10, m_disp.get_width()-20, m_disp.get_height()-20),
            frame, window::flags::border |window::flags::selectborder );
    button btn( rectangle(40, 20, 400 , 40), layout(), win );
    seekbar seek( rectangle(40, 75, 400 , 40), layout(), win );
    editbox edit( rectangle(40, 150, 400 , 30), layout(), win );
    choice_menu choice1( rectangle(40, 220, 400, 250), layout(), win, choice_menu::style::normal );
    edit.readonly(true);
    m_edit = &edit;
    seek.value( 50 );
    btn.caption("Button");
    edit.value("Text edit value");
    choice1.items( menu1 );
    //Connect windows callback to the main window
    win.connect(std::bind(&tft_livedemo::window_callback,this,std::placeholders::_1), event::evtype::EV_KEY);
    win.connect(std::bind(&tft_livedemo::window_callback,this,std::placeholders::_1), event::evtype::EV_TOUCH);
    btn.connect(std::bind(&tft_livedemo::on_click,this,std::placeholders::_1),event::evtype::EV_CLICK);
    btn.pushkey( gfx::input::kbdcodes::enter);
    choice1.connect(std::bind(&tft_livedemo::on_select_item,this,std::placeholders::_1),event::evtype::EV_CLICK);
    seek.connect(std::bind(&tft_livedemo::on_seek_change,this,std::placeholders::_1),event::evtype::EV_CLICK);
    frame.set_focus( &win );
    frame.execute();
}
void tft_livedemo::gdi_test()
{
    static unsigned int img[16*16];
    for( int x=0;x<16;x++ )
    for( int y=0;y<16;y++ )
    {
        if( y == 8 || y==0 || x==0 ||y==15||x==15)
            img[x + 16*y ] = gfx::rgb( 255 ,0, 255 );
        else
            img[x + 16*y ] = gfx::rgb( 255,255,0 );
    }
    m_disp.power_ctl( gfx::drv::power_ctl_t::on );
    gfx::disp::gdi gdi(m_disp);
    gdi.clear();
    ostick_t tbeg = isix::get_jiffies();
    static_cast<void>(tbeg);
    dbg_info("TIME=%u", isix::get_jiffies() - tbeg );
    m_disp.fill(50,150,30,30, gfx::rgb(127,0,255));
    m_disp.blit( 20, 40, 16, 16, 0, img );
    tbeg = isix::get_jiffies();
    dbg_info("BLIT=%u", isix::get_jiffies() - tbeg );
    dbg_info( "PIX1=%04X %04x", m_disp.get_pixel( 1, 1 ) ,  gfx::rgb(0,255,0)  );
    dbg_info( "PIX2=%04X %04x", m_disp.get_pixel(140,140 ), gfx::rgb(255,0,0) );
    gdi.set_fg_color( gfx::rgb(64,255,45) );
    constexpr char txt[] = "Przykladowy napis";
    tbeg = isix::get_jiffies();
    gdi.draw_text(1,2, txt);
    dbg_info("string_time=%u", isix::get_jiffies() - tbeg );
    tbeg = isix::get_jiffies();
    gdi.draw_line( 1, 100, 100, 100 );
    dbg_info("line_time=%u", isix::get_jiffies() - tbeg );
    tbeg = isix::get_jiffies();
    gdi.draw_circle( 150, 150, 30 );
    dbg_info("circle_time=%u", isix::get_jiffies() - tbeg );
    tbeg = isix::get_jiffies();
    gdi.draw_ellipse( 50, 150, 30, 40 );
    dbg_info("ellipse_time=%u", isix::get_jiffies() - tbeg );
    dbg_info("text_width=%u", gdi.get_text_width(txt));
    tbeg = isix::get_jiffies();
    gdi.set_fg_color( gfx::rgb(255,0,0) );
    gdi.set_bg_color( gfx::rgb(0,0,0) );
    dbg_info("drawimage_time=%u", isix::get_jiffies() - tbeg );
}

}
