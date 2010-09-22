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
#include "i2c_master.h"
#include <foundation.h>
#include <system.h>
/* ------------------------------------------------------------------ */
#define I2C_SPEED 100000
#define TEMPSENSOR_I2CADDR 0x90
#define MCP9800_CONFIG_REG 1
#define MCP9800_TEMP_REG 0
#define MCP9800_RES_12BIT (3<<5)
//Led Port
#define LED_PORT GPIOE
#define LED_PIN 14
//Blinking time and prio
#define BLINK_TIME 500
/* ------------------------------------------------------------------ */

#define TASK_STK_SIZE 256
#define TASK_PRIO_LED 3
#define TASK_PRIO_TEMP 2
#define MEASURE_WAIT_TIME 1000
/* ------------------------------------------------------------------ */
//Temp and tem/10 structure
struct temp
{
	short t;		//temp
	short ft;		// .temp fract
};

struct msg
{
	struct temp t;
	int errno;
};


/* ------------------------------------------------------------------ */
//Initialize the temperature sensor
static int tempsensor_init(void)
{
	static const uint8_t tmp[] = { MCP9800_CONFIG_REG , MCP9800_RES_12BIT };	//Config register
	return i2cm_transfer_7bit(TEMPSENSOR_I2CADDR,tmp,sizeof(tmp),NULL,0);
}

/* ------------------------------------------------------------------ */
//Get temperature
static int tempsensor_get(struct temp *t)
{
	static const unsigned char temp_reg = MCP9800_TEMP_REG;
	static unsigned char temp[2];
	int ecode;
	ecode = i2cm_transfer_7bit(TEMPSENSOR_I2CADDR,&temp_reg,sizeof(temp_reg),temp,sizeof(temp));
	if(ecode>=0)
	{
		t->t = temp[0];
		t->ft = (((int)(temp[1]>>4))*10)/16;
	}
	return ecode;
}
/* ------------------------------------------------------------------ */
//Display temperature
static void display_temp(int x,int y, struct temp *t, const char *str)
{
	char buf[16];
	tiny_snprintf( buf,sizeof(buf),"%d.%d %s", t->t, t->ft, str );
	nlcd_put_string(buf,x,y);
}

/* ------------------------------------------------------------------ */
ISIX_TASK_FUNC(temp_read_task,entry_params)
{
	fifo_t *temp_fifo = (fifo_t*)entry_params;
	struct msg msg;
	nlcd_init();
	//Initialize i2c library
	i2cm_init(I2C_SPEED);
	msg.errno = tempsensor_init();
	if(msg.errno<0)
	{
		isix_fifo_write( temp_fifo, &msg, ISIX_TIME_INFINITE );
		isix_task_delete(NULL);
	}
	for(;;)
	{
		msg.errno =  tempsensor_get(&msg.t);
		isix_fifo_write( temp_fifo, &msg, ISIX_TIME_INFINITE );
		isix_wait_ms(MEASURE_WAIT_TIME);
	}
}
/* ------------------------------------------------------------------ */
/** Display server task */
static ISIX_TASK_FUNC(display_srv_task, entry_params)
{
	fifo_t *temp_fifo = (fifo_t*)entry_params;
	struct msg msg;
	bool bstate = false;
	//Initialize LCD
	nlcd_init();
	//Put welcome string
	nlcd_put_string( "www.boff.pl", 0, 0 );
	nlcd_put_string( "Sensor Temp:", 0, 1 );
	for(;;)
	{
		//Read data from fifo
		if(isix_fifo_read( temp_fifo, &msg,ISIX_TIME_INFINITE )==ISIX_EOK)
		{
			if(msg.errno>=0)
				display_temp( 0, 2, &msg.t,"C     " );
			else
				nlcd_put_string("I2c Error",0,2);
		}
		if(bstate) nlcd_put_string("*",13,0);
		else nlcd_put_string(" ",13,0);
		bstate=!bstate;
	}
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
int main(void)
{
	//Create ISIX blinking task
	isix_task_create( blinking_task, NULL,
			ISIX_PORT_SCHED_MIN_STACK_DEPTH, TASK_PRIO_LED
	);
	fifo_t *temp_fifo = isix_fifo_create( 10, sizeof(struct msg) );
	if(temp_fifo)
	{
		//Create isix task
		isix_task_create(temp_read_task,temp_fifo,TASK_STK_SIZE,TASK_PRIO_TEMP);
		isix_task_create(display_srv_task,temp_fifo,TASK_STK_SIZE,TASK_PRIO_TEMP);
	}
	isix_start_scheduler();
	//Start the sheduler
}
/* ------------------------------------------------------------------ */
