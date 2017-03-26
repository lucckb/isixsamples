/* ------------------------------------------------------------------ */
/*
 * keyboard.cpp
 * Keyboyard send event thread class
 *  Created on: 2010-01-09
 *      Author: lucck
 */
/* ------------------------------------------------------------------ */
#include <stm32lib.h>
#include "graph_key.hpp"
#include "display_server.hpp"
#include "images.hpp"
/* ------------------------------------------------------------------ */
//The application namespace
namespace app
{

/* ------------------------------------------------------------------ */
//Unnamed namespace
namespace
{
	//Keyboyard port
	GPIO_TypeDef * const KEY_PORT = GPIOE;

	//OK button
	const unsigned KEY_OK_BIT = 8;
	const unsigned KEY_OK = 0x01;

	//UP button
	const unsigned KEY_UP_BIT = 9;
	const unsigned KEY_UP = 0x02;

	//DOWN button
	const unsigned KEY_DOWN_BIT = 10;
	const unsigned KEY_DOWN = 0x04;

	//RIGHT button
	const unsigned KEY_RIGHT_BIT = 11;
	const unsigned KEY_RIGHT = 0x08;

	//LEFT button
	const unsigned KEY_LEFT_BIT = 12;
	const unsigned KEY_LEFT = 0x10;

	//The kbd mask
	const unsigned KEY_MASK = 0x1f;

	//The Delay
	const unsigned DELAY_TIME = 25;

	//Get the key function
	inline unsigned short get_key()
	{
		return (~KEY_PORT->IDR >> KEY_OK_BIT) & KEY_MASK;
	}
}

/* ------------------------------------------------------------------ */
//Constructor
graph_key::graph_key(display_server &_srv) 
	: disp_srv(_srv)
{
	using namespace stm32;
	//Enable PE in APB2
	RCC->APB2ENR |= RCC_APB2Periph_GPIOE;
	//Set GPIO as inputs
	gpio_config(KEY_PORT,KEY_OK_BIT,GPIO_MODE_INPUT,GPIO_CNF_IN_FLOAT);
	gpio_config(KEY_PORT,KEY_UP_BIT,GPIO_MODE_INPUT,GPIO_CNF_IN_FLOAT);
	gpio_config(KEY_PORT,KEY_DOWN_BIT,GPIO_MODE_INPUT,GPIO_CNF_IN_FLOAT);
	gpio_config(KEY_PORT,KEY_LEFT_BIT,GPIO_MODE_INPUT,GPIO_CNF_IN_FLOAT);
	gpio_config(KEY_PORT,KEY_RIGHT_BIT,GPIO_MODE_INPUT,GPIO_CNF_IN_FLOAT);
}
/* ------------------------------------------------------------------ */
//Task/thread method
void graph_key::main()
{
	//Previous key variable
	static short p_key = -1;
	//Graphics message class
	static graph_msg gmsg;
	//Text message class
	static text_msg tmsg("Wcisnales OK");
	for(;;)
	{
		//Get key
		short key = get_key();
		//Check if any key is pressed
		if(key!=0 && p_key==0)
		{
			switch(key)
			{
			case KEY_OK:
				disp_srv.send_message(tmsg);
				break;
			case KEY_LEFT:
				gmsg.set_image(images::isixlogo_bmp);
				disp_srv.send_message(gmsg);
				break;
			case KEY_RIGHT:
				gmsg.set_image(images::disk_bmp);
				disp_srv.send_message(gmsg);
				break;
			case KEY_UP:
				gmsg.set_image(images::printer_bmp);
				disp_srv.send_message(gmsg);
				break;
			case KEY_DOWN:
				gmsg.set_image(images::scriba_bmp);
				disp_srv.send_message(gmsg);
				break;
			}
		}
		//Previous key assignement
		p_key = key;
		//Wait short time
		isix::wait_ms( DELAY_TIME );
	}
}
/* ------------------------------------------------------------------ */
}
/* ------------------------------------------------------------------ */
