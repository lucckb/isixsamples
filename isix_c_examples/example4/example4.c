/* ------------------------------------------------------------------ */
/*
 * example2.c
 *
 *  Created on: 19-09-2010
 *      Author: lucck
 */
/* ------------------------------------------------------------------ */
#include <isix.h>
#include "nokia_lcd.h"
#include <foundation.h>
#include <system.h>
/* ------------------------------------------------------------------ */
#define LED_PORT GPIOE				/* Led port def */
#define LED_PIN 14					/* Blink led pin */
#define BLINK_TIME 500				/* Blink time led */
/* ------------------------------------------------------------------ */
#define TASK_STK_SIZE 256			/* measure task stack size */
#define TASK_PRIO_LED 3				/* led task prio */
#define TASK_PRIO_KEY 3			    /* measure task prio */
#define TASK_PRIO_DISP 2			/* Display task prio */
#define KEY_SCAN_INTERVAL 9
/* ------------------------------------------------------------------ */

//Key definition
typedef char key_t;

/* ------------------------------------------------------------------ */
#define KEYSCAN_PORT GPIOE				/* Keyboard port */
#define KEYSCAN_ROW_PINS 0xf0			/* Row bitmask */
#define KEYSCAN_COL_PINS 0x0f			/* Col bitmask */
#define KEYSCAN_FIRST_ROW 0x10			/* First row def */
#define KEYSCAN_LAST_ROW 0x80			/* Last row def */
#define KEYSCAN_COL1_BIT 0x01			/* Cols and rows bit defs */
#define KEYSCAN_COL2_BIT 0x02
#define KEYSCAN_COL3_BIT 0x04
#define KEYSCAN_COL4_BIT 0x08
#define KEYSCAN_ROW1_BIT 0x10
#define KEYSCAN_ROW2_BIT 0x20
#define KEYSCAN_ROW3_BIT 0x40
#define KEYSCAN_ROW4_BIT 0x80

//Initialize the port configuration
static void keyscan_init(void)
{
	RCC->APB2ENR |= RCC_APB2Periph_GPIOE;
	//Input push pull +GND
	gpio_config_ext( KEYSCAN_PORT, KEYSCAN_COL_PINS, GPIO_MODE_INPUT, GPIO_CNF_IN_PULLUP );
	gpio_clr_mask(KEYSCAN_PORT, KEYSCAN_COL_PINS);
	//Output row to zero
	gpio_config_ext( KEYSCAN_PORT, KEYSCAN_ROW_PINS, GPIO_MODE_10MHZ, GPIO_CNF_GPIO_PP );
	gpio_clr_mask(KEYSCAN_PORT, KEYSCAN_ROW_PINS);
}

/* ------------------------------------------------------------------ */
//Translate row/col to key val
static inline char key_translate(int row, int col)
{
	switch(row)		/* Select row */
	{
	case KEYSCAN_ROW1_BIT:
		switch(col)	/* Select col */
		{
			case KEYSCAN_COL1_BIT: return '1';
			case KEYSCAN_COL2_BIT: return '2';
			case KEYSCAN_COL3_BIT: return '3';
			case KEYSCAN_COL4_BIT: return 'A';
		}
	break;
	case KEYSCAN_ROW2_BIT:
		switch(col) /* Select col */
		{
			case KEYSCAN_COL1_BIT: return '4';
			case KEYSCAN_COL2_BIT: return '5';
			case KEYSCAN_COL3_BIT: return '6';
			case KEYSCAN_COL4_BIT: return 'B';
		}
	break;
	case KEYSCAN_ROW3_BIT:
		switch(col) /* Select col */
		{
			case KEYSCAN_COL1_BIT: return '7';
			case KEYSCAN_COL2_BIT: return '8';
			case KEYSCAN_COL3_BIT: return '9';
			case KEYSCAN_COL4_BIT: return 'C';
		}
		break;
		case KEYSCAN_ROW4_BIT:
		switch(col) /* Select col */
		{
			case KEYSCAN_COL1_BIT: return '*';
			case KEYSCAN_COL2_BIT: return '0';
			case KEYSCAN_COL3_BIT: return '#';
			case KEYSCAN_COL4_BIT: return 'D';
		}
		break;
	}
	return 'x';	//Unknown key
}
/* ------------------------------------------------------------------ */
/** Temperature read task */
ISIX_TASK_FUNC(kbd_task,entry_params)
{
	//Initialize gpio port keyscan
	keyscan_init();
	//Fifo pointer
	fifo_t *key_fifo = (fifo_t*)entry_params;
	//Main loop
	for(;;)
	for(int row=KEYSCAN_FIRST_ROW;row<=KEYSCAN_LAST_ROW ;row<<=1)
	{
		//Set row output
	gpio_set_clr_mask( KEYSCAN_PORT, row, KEYSCAN_ROW_PINS );
		//Wait one tick before read
		isix_wait(1);
		int col =gpio_get_mask( KEYSCAN_PORT, KEYSCAN_COL_PINS);
		if(col>0)
		{
			//If col selected translate key and sent to disp
			key_t key = key_translate( row, col );
			isix_fifo_write( key_fifo, &key, isix_ms2tick(KEY_SCAN_INTERVAL));
		}
		//Wait one ms
		isix_wait_ms(KEY_SCAN_INTERVAL);
	}
}
/* ------------------------------------------------------------------ */
/** Display server task */
static ISIX_TASK_FUNC(display_srv_task, entry_params)
{
	fifo_t *temp_fifo = (fifo_t*)entry_params;
	key_t key;
	nlcd_init();			//Initialize LCD
	//Put welcome string
	nlcd_put_string( "www.boff.pl", 0, 0 );
	nlcd_put_string( "Clicked key:", 0, 1 );
	for(;;)
	{
		//Read data from fifo
		if(isix_fifo_read( temp_fifo, &key, ISIX_TIME_INFINITE )==ISIX_EOK)
		{
			nlcd_set_position(6,2);
			nlcd_put_char(key);
		}
	}
}
/* ------------------------------------------------------------------ */
/** Blinking led task function */
static ISIX_TASK_FUNC(blinking_task, entry_param)
{
	RCC->APB2ENR |= RCC_APB2Periph_GPIOE;
 gpio_config(LED_PORT,LED_PIN,GPIO_MODE_10MHZ,GPIO_CNF_GPIO_PP);
	for(;;)
	{
		//Enable LED
	gpio_clr( LED_PORT, LED_PIN );
		//Wait time
		isix_wait_ms( BLINK_TIME );
		//Disable LED
	gpio_set( LED_PORT, LED_PIN );
		//Wait time
		isix_wait_ms( BLINK_TIME );
	}
}
/* ------------------------------------------------------------------ */
/** Main func */
int main(void)
{
	//Create ISIX blinking task
	isix_task_create( blinking_task, NULL,ISIX_PORT_SCHED_MIN_STACK_DEPTH, TASK_PRIO_LED);
	//Create fifo msgs
	fifo_t *key_fifo = isix_fifo_create( 10, sizeof(key_t) );
	if(key_fifo)
	{
		//Create isix tasks (key and disp)
		isix_task_create(kbd_task,key_fifo,TASK_STK_SIZE,TASK_PRIO_KEY);
		isix_task_create(display_srv_task,key_fifo,TASK_STK_SIZE,TASK_PRIO_DISP);
	}
	isix_start_scheduler();
	//Start the sheduler
}
/* ------------------------------------------------------------------ */
