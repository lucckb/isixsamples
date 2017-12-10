/* ------------------------------------------------------------------ */
/*
 * ledkey.cpp
 *
 *  Created on: 2010-01-02
 *      Author: lucck
 */
/* ------------------------------------------------------------------ */
#include "ledkey.hpp"
#include <isix.h>
#include <stm32lib.h>
/* ------------------------------------------------------------------ */
namespace app
{
/* ------------------------------------------------------------------ */
//Unnamed namespace
namespace
{
	GPIO_TypeDef * const LED_PORT = GPIOE;
	GPIO_TypeDef * const KEY_PORT = GPIOE;
	const unsigned LED_PIN = 15;
	const unsigned KEY_PIN = 8;
	const unsigned DELAY_TIME = 25;
}

/* ------------------------------------------------------------------ */
//Default constructor initialize GPIO and var
ledkey::ledkey()
	:	is_enabled(false)
{
	using namespace stm32;
	//Enable PE in APB2
	RCC->APB2ENR |= RCC_APB2Periph_GPIOE;
	gpio_config(LED_PORT,LED_PIN,GPIO_MODE_10MHZ,GPIO_CNF_GPIO_PP);
}

/* ------------------------------------------------------------------ */
void ledkey::main() noexcept
{
	//Last key state
	bool p_state = true;
	//Task/thread main loop
	while(true)
	{
		//Change state on rising edge
		if(stm32::gpio_get(KEY_PORT, KEY_PIN) && !p_state)
		{
			is_enabled = !is_enabled;
		}
		//Get previous state
		p_state =stm32::gpio_get(KEY_PORT, KEY_PIN);
		//If enabled change state
		if(is_enabled) stm32::gpio_clr( LED_PORT, LED_PIN );
        else stm32::gpio_set( LED_PORT, LED_PIN );
		//Wait short time
		isix::wait_ms( DELAY_TIME );
	}
}

/* ------------------------------------------------------------------ */
}
