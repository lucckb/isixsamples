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
#include <lwip/init.h>
#include <foundation/dbglog.h>
#include <usart_simple.h>
#include "config.h"
#include <stm32rcc.h>
#include <stm32system.h>
#include <stm32gpio.h>
#include <stm32_eth.h>
#include <stdbool.h>
#include "tcpecho/tcpecho.h"
#include <eth/ethernetif.h>
#include <lwip/tcpip.h>
#include <lwip/dhcp.h>
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

/**
  * @brief  Initializes the lwIP stack
  * @param  None
  * @retval None
  *
  */

static void netif_callback( struct netif *netif )
{
	if( netif->flags & NETIF_FLAG_LINK_UP )
	{
		dbprintf("LINK-UP");
	}
	else
	{
		dbprintf("LINK-DOWN");
	}
}

static void tcp_eth_init(void)
{
  struct ip_addr ipaddr;
  struct ip_addr netmask;
  struct ip_addr gw;
  uint8_t macaddress[6]={0x00,0xA4,0xA5,0x10,0x20,0x30};

  netif_init();
  tcpip_init( NULL, NULL );

  struct netif *netif =  stm32_emac_netif_create( macaddress );
  if(!netif)
  {
	  isix_bug("Unable to create network interface");
  }
#if LWIP_DHCP
  ipaddr.addr = 0;
  netmask.addr = 0;
  gw.addr = 0;

#else
  IP4_ADDR(&ipaddr, 192, 168, 16, 222);
  IP4_ADDR(&netmask, 255, 255, 255, 0);
  IP4_ADDR(&gw, 192, 168, 16, 1);
#endif

  /* - netif_add(struct netif *netif, struct ip_addr *ipaddr,
            struct ip_addr *netmask, struct ip_addr *gw,
            void *state, err_t (* init)(struct netif *netif),
            err_t (* input)(struct pbuf *p, struct netif *netif))

   Adds your network interface to the netif_list. Allocate a struct
  netif and pass a pointer to this structure as the first argument.
  Give pointers to cleared ip_addr structures when using DHCP,
  or fill them with sane numbers otherwise. The state pointer may be NULL.

  The init function pointer must point to a initialization function for
  your ethernet netif interface. The following code illustrates it's use.*/
  netif_add(netif, &ipaddr, &netmask, &gw, NULL, &stm32_emac_if_init_callback, &tcpip_input);
  netif_set_link_callback( netif, netif_callback );
  /*  Registers the default network interface.*/
  netif_set_default(netif);

#if LWIP_DHCP
  /*  Creates a new DHCP client for this interface on the first call.
  Note: you must call dhcp_fine_tmr() and dhcp_coarse_tmr() at
  the predefined regular intervals after starting the client.
  You can peek in the netif->dhcp struct for the actual DHCP status.*/
  dhcp_start(netif);
#endif

  /*  When the netif is fully configured this function must be called.*/
  netif_set_up(netif);

}

/* ------------------------------------------------------------------ */

//App main entry point
int main(void)
{
	dblog_init( usartsimple_putc, NULL, usartsimple_init,
			USART2,115200,true, CONFIG_PCLK1_HZ, CONFIG_PCLK2_HZ );
	//receive_test();

	//Create ISIX blinking task
	isix_task_create( blinking_task, NULL,
			ISIX_PORT_SCHED_MIN_STACK_DEPTH, BLINKING_TASK_PRIO, 0 );
	 
	dbprintf("Hello from TCPIP stack example");
	//Initialize the tcpip library
	tcp_eth_init();
	tcpecho_init();
	//Start the isix scheduler
	isix_start_scheduler();
}


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
