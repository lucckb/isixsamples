/* ------------------------------------------------------------------ */
/*
 * blinker.cpp
 * The Led receiver class (Blink LED1 on the STM32 butterfly)
 *  Created on: 2010-01-02
 *      Author: lucck
 */
/* ------------------------------------------------------------------ */
#include "led_receiver.hpp"
#include "usart_buffered.hpp"
#include <stm32f10x_lib.h>
#include <isix.h>
/* ------------------------------------------------------------------ */
namespace app	//App namespace
{
/* ------------------------------------------------------------------ */
//Unnamed namespace
namespace
{
	GPIO_TypeDef * const LED_PORT = GPIOE;
	const unsigned LED1_PIN = 14;
	const unsigned LED2_PIN = 15;
	const unsigned BLINK_TIME = 500;
}

/* ------------------------------------------------------------------ */
//Default constructor, construct base object
led_receiver::led_receiver(dev::usart_buffered &_serial)
	:task_base(STACK_SIZE,TASK_PRIO),serial(_serial)
{
	//Enable PE in APB2
	RCC->APB2ENR |= RCC_APB2Periph_GPIOE;
	io_config(LED_PORT,LED1_PIN,GPIO_MODE_10MHZ,GPIO_CNF_GPIO_PP);
	io_config(LED_PORT,LED2_PIN,GPIO_MODE_10MHZ,GPIO_CNF_GPIO_PP);
	io_set( LED_PORT, LED2_PIN );
	io_set( LED_PORT, LED1_PIN );
}


/* ------------------------------------------------------------------ */
//Main task/thread function
void led_receiver::main()
{
	while(true)
	{
		unsigned char c;
		//Receive data from serial
		if(serial.getchar(c)==isix::ISIX_EOK)
		{
			//Check for received char
			switch(c)
			{
			//On led 1
			case 'a':
			case 'A':
				io_clr( LED_PORT, LED1_PIN );
				break;
			//Off led 1
			case 'b':
			case 'B':
				io_set( LED_PORT, LED1_PIN );
				break;
			//On led 2
			case 'c':
			case 'C':
				io_clr( LED_PORT, LED2_PIN );
				break;
			//Off led 2
			case 'd':
			case 'D':
				io_set( LED_PORT, LED2_PIN );
				break;
			}
		}
	}
}
/* ------------------------------------------------------------------ */
}
/* ------------------------------------------------------------------ */
