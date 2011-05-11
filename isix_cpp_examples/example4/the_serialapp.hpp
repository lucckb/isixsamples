/*
 * the_serialapp.hpp
 * The Application class decl
 *  Created on: 2010-01-03
 *      Author: lucck
 */
/* ------------------------------------------------------------------ */
#ifndef THE_SERIALAPP_HPP_
#define THE_SERIALAPP_HPP_
/* ------------------------------------------------------------------ */

#include "led_receiver.hpp"
#include "usart_buffered.hpp"
#include "key_transmitter.hpp"

/* ------------------------------------------------------------------ */
//Application namespace
namespace app
{

//The application class
class the_serialapp
{
public:
	//App Constructor
	the_serialapp(): usart(USART2),ledrcv(usart),keytran(usart)
	{}
private:
	//Serial device
	dev::usart_buffered usart;

	//The blinker class
	led_receiver ledrcv;

	//The key transmitter class
	key_transmitter keytran;


};

/* ------------------------------------------------------------------ */
}
/* ------------------------------------------------------------------ */
#endif /* THE_APPLICATION_HPP_ */
/* ------------------------------------------------------------------ */
