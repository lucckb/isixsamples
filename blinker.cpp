/* ------------------------------------------------------------------ */
/*
 * blinker.cpp
 * The blinker class (Blink LED1 on the STM32 butterfly)
 *  Created on: 2010-01-02
 *      Author: lucck
 */
/* ------------------------------------------------------------------ */
#include "blinker.hpp"
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
	const unsigned LED_PIN = 14;
	const unsigned BLINK_TIME = 500;
}

/* ------------------------------------------------------------------ */
//Default constructor, construct base object
blinker::blinker():task_base(STACK_SIZE,TASK_PRIO)
{
	//Enable PE in APB2
	RCC->APB2ENR |= RCC_APB2Periph_GPIOE;
	io_config(LED_PORT,LED_PIN,GPIO_MODE_10MHZ,GPIO_CNF_GPIO_PP);
}


/* ------------------------------------------------------------------ */
//Main task/thread function
void blinker::main()
{
	while(true)
	{
		//Enable LED
		io_clr( LED_PORT, LED_PIN );
		//Wait time
		isix::isix_wait( isix::isix_ms2tick(BLINK_TIME) );
		//Disable LED
		io_set( LED_PORT, LED_PIN );
		//Wait time
		isix::isix_wait( isix::isix_ms2tick(BLINK_TIME) );
	}
}
/* ------------------------------------------------------------------ */
}
/* ------------------------------------------------------------------ */
