/* ------------------------------------------------------------------ */
/*
 * blinker.cpp
 * The Led receiver class (Blink LED1 on the STM32 butterfly)
 *  Created on: 2010-01-02
 *      Author: lucck
 */
/* ------------------------------------------------------------------ */
#include "led_receiver.hpp"
#include <isixdrv/usart_buffered.hpp>
#include <stm32lib.h>
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
led_receiver::led_receiver(stm32::dev::usart_buffered &_serial)
	: serial(_serial)
{
	using namespace stm32;
	//Enable PE in APB2
	RCC->APB2ENR |= RCC_APB2Periph_GPIOE;
	gpio_config(LED_PORT,LED1_PIN,GPIO_MODE_10MHZ,GPIO_CNF_GPIO_PP);
	gpio_config(LED_PORT,LED2_PIN,GPIO_MODE_10MHZ,GPIO_CNF_GPIO_PP);
    gpio_set( LED_PORT, LED2_PIN );
    gpio_set( LED_PORT, LED1_PIN );
}


/* ------------------------------------------------------------------ */
//Main task/thread function
void led_receiver::main()
{
	while(true)
	{
		char c;
		//Receive data from serial
		if(serial.getchar(c)==ISIX_EOK)
		{
			//Check for received char
			switch(c)
			{
			//On led 1
			case 'a':
			case 'A':
				stm32::gpio_clr( LED_PORT, LED1_PIN );
				break;
			//Off led 1
			case 'b':
			case 'B':
                stm32::gpio_set( LED_PORT, LED1_PIN );
				break;
			//On led 2
			case 'c':
			case 'C':
				stm32::gpio_clr( LED_PORT, LED2_PIN );
				break;
			//Off led 2
			case 'd':
			case 'D':
                stm32::gpio_set( LED_PORT, LED2_PIN );
				break;
			}
		}
	}
}
/* ------------------------------------------------------------------ */
}
/* ------------------------------------------------------------------ */
