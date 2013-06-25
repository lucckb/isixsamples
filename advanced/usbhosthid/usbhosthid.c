/* ------------------------------------------------------------------ */
/*
 * example1.c
 *
 * USB HID host sample
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
#include <usbhidkbd.hpp>

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


int _open( const char *a, int b )
{
	return 0;
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
	dbprintf("Hello from USBHOSTHID example");

	RCC->APB2ENR |= RCC_APB2Periph_GPIOD;
	gpio_config(GPIOD,15,GPIO_MODE_10MHZ,GPIO_CNF_GPIO_PP);
	gpio_set( GPIOD, 15 );
	/* Initialize the usb serial */
	stm32_usbhost_init();
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
	tiny_printf("[R0=%08lx]\t[R1=%08lx]\t[R2=%08lx]\t[R3=%08lx]\r\n", SP[0],SP[1],SP[2],SP[3]);
	tiny_printf("[R12=%08lx]\t[LR=%08lx]\t[PC=%08lx]\t[PSR=%08lx]\r\n",SP[4],SP[5],SP[6],SP[7]);
	const unsigned long rBFAR = (*((volatile unsigned long *)(0xE000ED38)));
	const unsigned long rCFSR = (*((volatile unsigned long *)(0xE000ED28)));
	const unsigned long rHFSR = (*((volatile unsigned long *)(0xE000ED2C)));
	const unsigned long rDFSR = (*((volatile unsigned long *)(0xE000ED30)));
	const unsigned long rAFSR = (*((volatile unsigned long *)(0xE000ED3C)));
	tiny_printf("[BAFR=%08lx]\t[CFSR=%08lx]\t[HFSR=%08lx]\t[DFSR=%08lx]\r\n",rBFAR,rCFSR,rHFSR,rDFSR);
	tiny_printf("[AFSR=%08lx]\r\n", rAFSR);
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
