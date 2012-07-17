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
#define I2C_SPEED 100000			/*I2c speed on the bus */
#define TEMPSENSOR_I2CADDR 0x90		/*I2c temp sensor addr */
#define MCP9800_CONFIG_REG 1		/* configuration register */
#define MCP9800_TEMP_REG 0			/* temperature register */
#define MCP9800_RES_12BIT (3<<5)	/* 12 bit resolution val */
#define LED_PORT GPIOE				/* Led port def */
#define LED_PIN 14					/* Blink led pin */
#define BLINK_TIME 500				/* Blink time led */
/* ------------------------------------------------------------------ */
#define TASK_STK_SIZE 256			/* measure task stack size */
#define TASK_PRIO_LED 3				/* led task prio */
#define TASK_PRIO_TEMP 2			/* measure task prio */
#define MEASURE_WAIT_TIME 500		/* Measure interval */
/* ------------------------------------------------------------------ */
//Temperature fractional structure
struct temp
{
	short t;		//temp
	short ft;		// .temp fract
};

//Message structure
struct msg
{
	struct temp t;		//Temperature
	int errno;			//Error code
};


/* ------------------------------------------------------------------ */
//Configure temperature sensor to 12 bit resolution
static int tempsensor_init(void)
{
	static const uint8_t tmp[] = { MCP9800_CONFIG_REG , MCP9800_RES_12BIT };	//Config register
	//Send configuration data
	return i2cm_transfer_7bit(TEMPSENSOR_I2CADDR,tmp,sizeof(tmp),NULL,0);
}

/* ------------------------------------------------------------------ */
//Get current temperature and fill the structure
static int tempsensor_get(struct temp *t)
{
	static const unsigned char temp_reg = MCP9800_TEMP_REG;
	static unsigned char temp[2];
	int ecode;
	//Read the temperature
	ecode = i2cm_transfer_7bit(TEMPSENSOR_I2CADDR,&temp_reg,sizeof(temp_reg),temp,sizeof(temp));
	//Convert to integer
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
/** Temperature read task */
ISIX_TASK_FUNC(temp_read_task,entry_params)
{
	fifo_t *temp_fifo = (fifo_t*)entry_params;
	struct msg msg;
	msg.errno = tempsensor_init();	//Init temp sensor
	if(msg.errno<0)	//If cfg fail
	{
		//Push error message and destroy task
		isix_fifo_write( temp_fifo, &msg, ISIX_TIME_INFINITE );
		isix_task_delete(NULL);
	}
	for(;;)
	{
		//Read temp
		msg.errno =  tempsensor_get(&msg.t);
		//Push data
		isix_fifo_write( temp_fifo, &msg, ISIX_TIME_INFINITE );
		//Wait for the next cycle
		isix_wait_ms(MEASURE_WAIT_TIME);
	}
}
/* ------------------------------------------------------------------ */
/** Display server task */
static ISIX_TASK_FUNC(display_srv_task, entry_params)
{
	fifo_t *temp_fifo = (fifo_t*)entry_params;
	struct msg msg;			//Message structure
	bool bstate = false;	//Previous state
	nlcd_init();			//Initialize LCD
	//Put welcome string
	nlcd_put_string( "www.boff.pl", 0, 0 );
	nlcd_put_string( "Sensor Temp:", 0, 1 );
	for(;;)
	{
		//Read data from fifo
		if(isix_fifo_read( temp_fifo, &msg,ISIX_TIME_INFINITE )==ISIX_EOK)
		{
			//Display temp or error
			if(msg.errno>=0)
				display_temp( 0, 2, &msg.t,"C     " );
			else
				nlcd_put_string("I2c Error    ",0,2);
		}
		//Blinking *
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
/** Main func  */
int main(void)
{
	//Create ISIX blinking task
	isix_task_create( blinking_task, NULL,
			ISIX_PORT_SCHED_MIN_STACK_DEPTH, TASK_PRIO_LED
	);
	//Create fifo msgs
	fifo_t *temp_fifo = isix_fifo_create( 10, sizeof(struct msg) );
	//Initialize i2c bus
	i2cm_init(I2C_SPEED);
	if(temp_fifo)
	{
		//Create isix tasks (temp and disp)
		isix_task_create(temp_read_task,temp_fifo,TASK_STK_SIZE,TASK_PRIO_TEMP);
		isix_task_create(display_srv_task,temp_fifo,TASK_STK_SIZE,TASK_PRIO_TEMP);
	}
	isix_start_scheduler();
	//Start the sheduler
}
/* ------------------------------------------------------------------ */
