/* ------------------------------------------------------------------ */
/*
 * example2.c
 *
 *  Created on: 19-09-2010
 *      Author: lucck
 */
/* ------------------------------------------------------------------ */
#include <isix.h>
#include "i2c_master.h"
#include <stm32system.h>
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
#define TASK_STK_SIZE 288			/* measure task stack size */
#define TASK_PRIO_LED 3				/* led task prio */
#define TASK_PRIO_TEMP 2			/* measure task prio */
#define MEASURE_WAIT_TIME 500		/* Measure interval */
/* ------------------------------------------------------------------ */
#define RGBCTRL_I2CADDR 0x00
/* ------------------------------------------------------------------ */
//Message structure
struct msg
{
	float t;			//Current temperature
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
static int tempsensor_get(float *t)
{
	static const unsigned char temp_reg = MCP9800_TEMP_REG;
	static unsigned char temp[2];
	int ecode;
	//Read the temperature
	ecode = i2cm_transfer_7bit(TEMPSENSOR_I2CADDR,&temp_reg,sizeof(temp_reg),temp,sizeof(temp));
	//Convert to integer
	if(ecode>=0)
	{
		*t = (float)temp[0] + (temp[1]>>4)/16.0f;
	}
	return ecode;
}

/* ------------------------------------------------------------------ */
#define RGB_INIT_NUMREG 3
#define RGB_INIT_NSEQ 2

//Initialize rgb controller
static int rgb_init(void)
{
	int errcode;
	static const uint8_t init_reg[RGB_INIT_NUMREG][RGB_INIT_NSEQ] =
	{ { 0, 0 }, { 1, 0 }, { 8, 255 } };
	for(int i=0;i<RGB_INIT_NUMREG;i++)
	{
	if(
		(errcode=i2cm_transfer_7bit(RGBCTRL_I2CADDR,init_reg[i],RGB_INIT_NSEQ ,NULL,0))
		< 0
        )
			return errcode;
	}
	return errcode;
}
/* ------------------------------------------------------------------ */
#define RGBA(r,g,b,a) (unsigned)(r) | ((unsigned)(g)<<8) | ((unsigned)(b)<<16) | ((unsigned)(a)<<24)
#define RGB(r,g,b) RGBA(r,g,b,0)

static int rgb_set_color (unsigned c)
{
	uint8_t txbytes[] =
	{
		0x82,	   //Color register
		(c)>>24,   //A
		(c)>>16,   //B
		(c),	   //R
		(c)>>8 	   //G
	};
	return i2cm_transfer_7bit(RGBCTRL_I2CADDR,txbytes,sizeof(txbytes),NULL,0);
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
#define COLD_VALUE 24.0f	/* Cold value */
#define HOT_VALUE 30.0f		/* Hot Value */
#define PWM_MAX 255			/* Maximum pwm value */
#define PWM_MAX_F 255.5
#define A_VALUE -0.1f		/* A value of quadratic equation */
#define FAIL_AMBER_VAL 200	/* Fail amber 4 PWM ratio */

/** Display server task */
static ISIX_TASK_FUNC(display_srv_task, entry_params)
{
	fifo_t *temp_fifo = (fifo_t*)entry_params;
	struct msg msg;			//Message structure
	//If init rgb fail terminate task
	if(rgb_init()<0)
	{
		isix_task_delete(NULL);
	}
	unsigned pcolor = 0;
	for(;;)
	{
		//Read data from fifo
		if(isix_fifo_read( temp_fifo, &msg,ISIX_TIME_INFINITE )==ISIX_EOK)
		{
			//Display temp or error
			if(msg.errno>=0)
			{
				//Calculate saturation
				int b = PWM_MAX_F * (HOT_VALUE - msg.t)/(HOT_VALUE-COLD_VALUE);
				int g = PWM_MAX_F * (A_VALUE * (msg.t-COLD_VALUE)*(msg.t-HOT_VALUE));
				int r = PWM_MAX_F * (msg.t-COLD_VALUE)/(HOT_VALUE-COLD_VALUE);
				if(b<0) b=0; else if(b>PWM_MAX) b=PWM_MAX;
				if(r<0) r=0; else if(r>PWM_MAX) r=PWM_MAX;
				if(g<0) g=0; else if(g>PWM_MAX) g=PWM_MAX;
				//Calculate color
				unsigned color = RGB(r,g,b);
				//Update britness only if color changed value
				if(color!=pcolor)
					rgb_set_color(color);
				pcolor = color;
			}
			else	// If fail enable only amber
				rgb_set_color(RGBA(0,0,0,FAIL_AMBER_VAL));
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
/** Main func  */
int main(void)
{
	//Create ISIX blinking task
	isix_task_create( blinking_task, NULL,ISIX_PORT_SCHED_MIN_STACK_DEPTH, TASK_PRIO_LED,0);
	//Create fifo msgs
	fifo_t *temp_fifo = isix_fifo_create( 10, sizeof(struct msg) );
	//Initialize i2c bus
	i2cm_init(I2C_SPEED);
	if(temp_fifo)
	{
		//Create isix tasks (temp and disp)
		isix_task_create(temp_read_task,temp_fifo,TASK_STK_SIZE,TASK_PRIO_TEMP,0);
		isix_task_create(display_srv_task,temp_fifo,TASK_STK_SIZE,TASK_PRIO_TEMP,0);
	}
	//Start the scheduler
	isix_start_scheduler();
}
/* ------------------------------------------------------------------ */
