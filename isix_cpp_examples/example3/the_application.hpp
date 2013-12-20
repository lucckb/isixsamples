/*
 * the_application.hpp
 * The Application class header
 *  Created on: 2010-01-03
 *      Author: lucck
 */
/* ------------------------------------------------------------------ */
#ifndef THE_APPLICATION_HPP_
#define THE_APPLICATION_HPP_
/* ------------------------------------------------------------------ */
#include "blinker.hpp"
#include "display_server.hpp"
#include "rtc_reader.hpp"
/* ------------------------------------------------------------------ */
//Application namespace
namespace app
{

//The application class
class the_application
{
public:
	//Constructor
	the_application();
	//Start
	void start() {
		led_blinker.start();
		dsrv.start();
		rtcr.start();
	}
private:
	//The blinker class
	ledblink led_blinker;
	//The display server class
	display_server dsrv;
	//The rtc reader
	rtc_reader rtcr;
};

/* ------------------------------------------------------------------ */
}
/* ------------------------------------------------------------------ */
#endif /* THE_APPLICATION_HPP_ */
/* ------------------------------------------------------------------ */
