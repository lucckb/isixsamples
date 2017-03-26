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
#include "graph_key.hpp"
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
	void start() {
		led_blinker.start();
		dsrv.start();
		keys.start();
	}
private:
	//The blinker class
	ledblink led_blinker;
	//The display server class
	display_server dsrv;
	//The keyboard task
	graph_key keys;
};

/* ------------------------------------------------------------------ */
}
/* ------------------------------------------------------------------ */
#endif /* THE_APPLICATION_HPP_ */
/* ------------------------------------------------------------------ */
