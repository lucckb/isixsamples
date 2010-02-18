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
#include "led_receiver.hpp"
#include "usart_buffered.hpp"
/* ------------------------------------------------------------------ */
//Application namespace
namespace app
{

//The application class
class the_serialapp
{
public:
	//Constructor
	the_serialapp(): usart(USART2),ledrcv(usart)
	{}
private:
	//Serial device
	dev::usart_buffered usart;

	//The blinker class
	led_receiver ledrcv;

};

/* ------------------------------------------------------------------ */
}
/* ------------------------------------------------------------------ */
#endif /* THE_APPLICATION_HPP_ */
/* ------------------------------------------------------------------ */
