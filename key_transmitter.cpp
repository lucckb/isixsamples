/* ------------------------------------------------------------------ */
/*
 * key_transmitter.cpp
 * The serial key transmitter demo
 *  Created on: 2010-02-21
 *      Author: lucck
 */
/* ------------------------------------------------------------------ */
#include "key_transmitter.hpp"
#include "usart_buffered.hpp"
/* ------------------------------------------------------------------ */
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
key_transmitter::key_transmitter(dev::usart_buffered &_serial) :
		task_base(STACK_SIZE,TASK_PRIO),serial(_serial)
{
	 using namespace stm32;
	 //Enable PE in APB2
	 RCC->APB2ENR |= RCC_APB2Periph_GPIOE;
	 //Set GPIO as inputs
	 io_config(KEY_PORT,KEY_OK_BIT,GPIO_MODE_INPUT,GPIO_CNF_IN_FLOAT);
	 io_config(KEY_PORT,KEY_UP_BIT,GPIO_MODE_INPUT,GPIO_CNF_IN_FLOAT);
	 io_config(KEY_PORT,KEY_DOWN_BIT,GPIO_MODE_INPUT,GPIO_CNF_IN_FLOAT);
	 io_config(KEY_PORT,KEY_LEFT_BIT,GPIO_MODE_INPUT,GPIO_CNF_IN_FLOAT);
	 io_config(KEY_PORT,KEY_RIGHT_BIT,GPIO_MODE_INPUT,GPIO_CNF_IN_FLOAT);
}

/* ------------------------------------------------------------------ */
//Main task - read joy key and sent it to RS232
void key_transmitter::main()
{
	//Previous key variable
	static short p_key = -1;
	serial.puts("Key A - enable LED1\r\n");
	serial.puts("Key B - disable LED1\r\n");
	serial.puts("Key C - enable LED2\r\n");
	serial.puts("Key D - disable LED2\r\n");
	serial.puts("Press joy on the stm32 butterfly\r\n");
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
				serial.puts("Key OK pressed\r\n");
				break;
			case KEY_LEFT:
				serial.puts("Key LEFT pressed\r\n");
				break;
			case KEY_RIGHT:
				serial.puts("Key RIGHT pressed\r\n");
				break;
			case KEY_UP:
				serial.puts("Key UP pressed\r\n");
				break;
			case KEY_DOWN:
				serial.puts("Key DOWN pressed\r\n");
				break;
			}
		}
		//Previous key assignement
		p_key = key;
		//Wait short time
		isix::isix_wait( isix::isix_ms2tick(DELAY_TIME) );
	}
}

/* ------------------------------------------------------------------ */


/* ------------------------------------------------------------------ */
}
