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
#include <stm32lib.h>
#include <dbglog.h>
#include <usart_simple.h>
#include "config.h"
#include <stm32rcc.h>
#include <stm32system.h>
#include <stm32gpio.h>
#include <stdbool.h>
#include <usbdevserial.h>
/* ------------------------------------------------------------------ */
//Led Port
#define LED_PORT GPIOE
#define LED_PIN 14

//Blinking time and prio
#define BLINK_TIME 500
#define BLINKING_TASK_PRIO 3

/* ------------------------------------------------------------------ */
/** Blinking led task function */
static ISIX_TASK_FUNC(blinking_task, entry_param)
{
	(void)entry_param;
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
static const char long_text[] =
		"With Kindles and ebooks on everyone's lips (sc. hands) nowadays, this might come as a surprise to some, but besides being a techie, I have also amassed quite a collection of actual books (mostly hardcover and first editions) in my personal library. I have always been reluctant to lend them out and the collection has grown so large now that it has become difficult to keep track of all of them. This is why I am looking for a modern solution to implement some professional-yet-still-home-sized library management. Ideally, this should include some cool features like RFID tags or NFC for keeping track of the books, finding and checking them out quickly, if I decide to lend one.";
/* ------------------------------------------------------------------ */
static ISIX_TASK_FUNC(cdc_task, entry_param)
{
	(void)entry_param;
	for(;;)
	{
		isix_wait_ms(3000);
		stm32_usbdev_write( long_text, sizeof(long_text)-1 );
	}
}

/* ------------------------------------------------------------------ */
//App main entry point
int main(void)
{
	dblog_init( usartsimple_putc, NULL, usartsimple_init,
			USART2,115200,true, PCLK1_HZ, PCLK2_HZ );

	//Create ISIX blinking task
	isix_task_create( blinking_task, NULL,
			ISIX_PORT_SCHED_MIN_STACK_DEPTH, BLINKING_TASK_PRIO
	);
	isix_task_create( cdc_task, NULL,
				1024, BLINKING_TASK_PRIO
		);
	dbprintf("Hello from CDCACM usb example");

	/* Initialize the usb serial */
	stm32_usbdev_serial_init(  );
    //Start the isix scheduler
	isix_start_scheduler();
}

/* ------------------------------------------------------------------ */
enum crash_mode
{
	CRASH_TYPE_USER=1,
	CRASH_TYPE_SYSTEM
};
/* ------------------------------------------------------------------ */
static inline void crash_info(enum crash_mode crash_type, unsigned long * SP)
{
	//Disable interrupt
	irq_disable();
	//Initialize usart simple no interrupt
	tiny_printf("\r\n\r\n ^^^^^^^^^^ CPU Crashed in [%s] mode!!! ARMv7m core regs: ^^^^^^^^^\r\n",
			crash_type==CRASH_TYPE_USER?"USER":"SYSTEM" );
	tiny_printf("[R0=%08x]\t[R1=%08x]\t[R2=%08x]\t[R3=%08x]\r\n", SP[0],SP[1],SP[2],SP[3]);
	tiny_printf("[R12=%08x]\t[LR=%08x]\t[PC=%08x]\t[PSR=%08x]\r\n",SP[4],SP[5],SP[6],SP[7]);
	const unsigned long rBFAR = (*((volatile unsigned long *)(0xE000ED38)));
	const unsigned long rCFSR = (*((volatile unsigned long *)(0xE000ED28)));
	const unsigned long rHFSR = (*((volatile unsigned long *)(0xE000ED2C)));
	const unsigned long rDFSR = (*((volatile unsigned long *)(0xE000ED30)));
	const unsigned long rAFSR = (*((volatile unsigned long *)(0xE000ED3C)));
	tiny_printf("[BAFR=%08x]\t[CFSR=%08x]\t[HFSR=%08x]\t[DFSR=%08x]\r\n",rBFAR,rCFSR,rHFSR,rDFSR);
	tiny_printf("[AFSR=%08x]\r\n", rAFSR);
	for(;;) wfi();
}
/* ------------------------------------------------------------------ */
void hard_fault_exception_vector(void) __attribute__((__interrupt__,naked));

/* ------------------------------------------------------------------ */
void hard_fault_exception_vector(void)
{
	unsigned long *sp;
	enum crash_mode cmode;
	//Check for SP or MSP
	asm(
		"TST LR, #4\n"
	    "ITTEE EQ\n"
	    "MRSEQ %[stackptr], MSP\n"
		"MOVEQ %[crashm],%[tsystem]\n"
	    "MRSNE %[stackptr], PSP\n"
		"MOVNE %[crashm],%[tuser]\n"
		: [stackptr] "=r"(sp), [crashm] "=r"(cmode):
		  [tuser]"I"(CRASH_TYPE_USER),[tsystem]"I"(CRASH_TYPE_SYSTEM)
		);
	//Print the crash info
	crash_info( cmode, sp );
}
