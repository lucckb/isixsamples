/* ------------------------------------------------------------------ */
/*
 * example1.c
 *
 * Blinking leds and Joystick with Nokia 3310 display
 * ISIX RTOS C example 1
 *
 *  Created on: 18-09-2010
 *      Author: lucck
 */
/* ------------------------------------------------------------------ */
#include <isix.h>
#include <stm32f10x_lib.h>
#include "nokia_lcd.h"
/* ------------------------------------------------------------------ */
//Led Port
#define LED_PORT GPIOE
#define LED_PIN 14

//Blinking time and prio
#define BLINK_TIME 500
#define BLINKING_TASK_PRIO 3
//Display task cfg
#define DISPLAY_TASK_PRIO 2
#define DISPLAY_TASK_STACK_SIZE 256
//KBD task cfg
#define KBD_TASK_PRIO 2
#define KBD_TASK_STACK_SIZE 256
//Keyboyard port
#define KEY_PORT  GPIOE
#define KEY_OK_BIT  8
#define KEY_OK  0x01
//UP button
#define KEY_UP_BIT  9
#define KEY_UP  0x02
//DOWN button
#define KEY_DOWN_BIT  10
#define KEY_DOWN  0x04
//RIGHT button
#define KEY_RIGHT_BIT  11
#define KEY_RIGHT  0x08
//LEFT button
#define KEY_LEFT_BIT  12
#define KEY_LEFT  0x10
//The kbd mask
#define KEY_MASK  0x1f
//The Delay
#define KBD_DELAY_TIME  25

typedef short key_t;
/* ------------------------------------------------------------------ */
//Get the key function
static inline unsigned short get_key()
{
		return (~KEY_PORT->IDR >> KEY_OK_BIT) & KEY_MASK;
}
/* ------------------------------------------------------------------ */
/** Blinking led task function */
static ISIX_TASK_FUNC(blinking_task, entry_param)
{
	RCC->APB2ENR |= RCC_APB2Periph_GPIOE;
	io_config(LED_PORT,LED_PIN,GPIO_MODE_10MHZ,GPIO_CNF_GPIO_PP);
	for(;;)
	{
		//Enable LED
		io_clr( LED_PORT, LED_PIN );
		//Wait time
		isix_wait_ms( BLINK_TIME );
		//Disable LED
		io_set( LED_PORT, LED_PIN );
		//Wait time
		isix_wait_ms( BLINK_TIME );
	}
}

/* ------------------------------------------------------------------ */
/** Display server task */
static ISIX_TASK_FUNC(display_srv_task, entry_params)
{
	fifo_t *kbd_fifo = (fifo_t*)entry_params;
	//Initialize LCD
	nlcd_init();
	//Put welcome string
	nlcd_put_string("www.boff.pl",0,0);
	nlcd_put_string("Joy state:",0,1);
	//Key variable read from
	key_t key;
	for(;;)
	{
		//Read data from fifo
		if(isix_fifo_read( kbd_fifo, &key,ISIX_TIME_INFINITE )==ISIX_EOK)
		{
			//Display text based on the key
			switch(key)
			{
			case KEY_LEFT:
				nlcd_put_string("** KEY LEFT **",0,2);
				break;
			case KEY_RIGHT:
				nlcd_put_string("** KEY RIGHT *",0,2);
				break;
			case KEY_UP:
				nlcd_put_string("** KEY UP **  ",0,2);
				break;
			case KEY_DOWN:
				nlcd_put_string("** KEY DOWN **",0,2);
				break;
			case KEY_OK:
				nlcd_put_string("** KEY OK **  ",0,2);
				break;
			}
		}
	}
}
/* ------------------------------------------------------------------ */
/** Keyboyard server task */
static ISIX_TASK_FUNC(keyboard_srv_task,entry_params)
{
	//Enable PE in APB2
	RCC->APB2ENR |= RCC_APB2Periph_GPIOE;
	//Set GPIO as inputs
	io_config(KEY_PORT,KEY_OK_BIT,GPIO_MODE_INPUT,GPIO_CNF_IN_FLOAT);
	io_config(KEY_PORT,KEY_UP_BIT,GPIO_MODE_INPUT,GPIO_CNF_IN_FLOAT);
	io_config(KEY_PORT,KEY_DOWN_BIT,GPIO_MODE_INPUT,GPIO_CNF_IN_FLOAT);
	io_config(KEY_PORT,KEY_LEFT_BIT,GPIO_MODE_INPUT,GPIO_CNF_IN_FLOAT);
	io_config(KEY_PORT,KEY_RIGHT_BIT,GPIO_MODE_INPUT,GPIO_CNF_IN_FLOAT);
	fifo_t *kbd_fifo = (fifo_t*)entry_params;
	//Previous key variable
	static key_t p_key = -1;
	for(;;)
	{
		//Get key
		key_t key = get_key();
		//Check if any key is pressed
		if(key!=0 && p_key==0)
		{
			isix_fifo_write( kbd_fifo, &key, ISIX_TIME_INFINITE );
		}
		//Previous key assignement
		p_key = key;
		//Wait short time
		isix_wait_ms( KBD_DELAY_TIME );
	}
}

/* ------------------------------------------------------------------ */
//App main entry point
int main(void)
{
	//Create ISIX blinking task
	isix_task_create( blinking_task, NULL,
			ISIX_PORT_SCHED_MIN_STACK_DEPTH, BLINKING_TASK_PRIO
	);
	fifo_t *kbd_fifo = isix_fifo_create( 10, sizeof(key_t) );
	if(kbd_fifo)
	{
		//Create the display serwer task
		isix_task_create(display_srv_task,kbd_fifo,DISPLAY_TASK_STACK_SIZE,DISPLAY_TASK_PRIO);

		//Create the keyboyard task
		isix_task_create(keyboard_srv_task, kbd_fifo, KBD_TASK_STACK_SIZE, KBD_TASK_PRIO);
	}
	//Start the isix scheduler
	isix_start_scheduler();
}
/* ------------------------------------------------------------------ */
