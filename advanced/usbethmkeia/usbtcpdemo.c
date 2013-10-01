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
#include "ethernetif.h"
#include <lwip/tcpip.h>
#include <lwip/dhcp.h>
#include <usbdevserial.h>
#include <stm32crashinfo.h>
/* ------------------------------------------------------------------ */
//Led Port
#define LED_PORT1 GPIOA
#define LED_PIN1 5
#define LED_PORT2 GPIOC
#define LED_PIN2 8
//Blinking time and prio
#define BLINK_TIME 500
#define BLINKING_TASK_PRIO 3

/* ------------------------------------------------------------------ */
/** Blinking led task function */
static ISIX_TASK_FUNC(blinking_task, entry_param)
{
	(void)entry_param;
	gpio_clock_enable( GPIOA, true );
	gpio_clock_enable( GPIOF, true );
	gpio_config(LED_PORT1,LED_PIN1,GPIO_MODE_10MHZ,GPIO_CNF_GPIO_PP);
	gpio_config(LED_PORT2,LED_PIN2,GPIO_MODE_10MHZ,GPIO_CNF_GPIO_PP);
	gpio_set( LED_PORT2, LED_PIN2 );
	for(;;)
	{
		//Enable LEDEunikova
		gpio_clr( LED_PORT1, LED_PIN1 );
		//Wait time
		isix_wait_ms( BLINK_TIME );
		//Disable LED
		gpio_set( LED_PORT1, LED_PIN1 );
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
#ifdef PDEBUG
//Crash info interrupt handler
void __attribute__((__interrupt__,naked)) hard_fault_exception_vector(void)
{
	cm3_hard_hault_regs_dump();
}
#endif
/* ------------------------------------------------------------------ */
static const char long_text[] =
		"With Kindles and ebooks on everyone's lips (sc. hands) nowadays, this might come as a surprise to some,"
		"but besides being a techie, I have also amassed quite a collection of actual books (mostly hardcover and "
		"first editions) in my personal library. I have always been reluctant to lend them out and the collection "
		 "has grown so large now that it has become difficult to keep track of all of them. This is why I am looking" ""
		 "for a modern solution to implement some professional-yet-still-home-sized library management. Ideally, this "
		 "should include some cool features like RFID tags or NFC for keeping track of the books, finding and checking "
		 "them out quickly, if I decide to lend one.\r\n";
static char tbuf[256];
/* ------------------------------------------------------------------ */
static ISIX_TASK_FUNC(cdc_task, entry_param)
{
	(void)entry_param;
	for(;;)
	{
		int ret = stm32_usbdev_serial_read(tbuf, sizeof(tbuf), 1000 );
		dbprintf("R: %d %s", ret, tbuf);
		if( ret >= 0)
		{
			tbuf[ret] = 0;
			ret = stm32_usbdev_serial_write(long_text, sizeof(long_text)-1, ISIX_TIME_INFINITE );
			dbprintf("W: %d %d", ret,  sizeof(long_text)-1);
		}
		else if( ret < 0 )
		{
			stm32_usbdev_wait_for_device_connected( 1000 );
		}
	}
}
/* ------------------------------------------------------------------ */
//App main entry point
int main(void)
{
	dblog_init( usartsimple_putc, NULL, usartsimple_init,
			USART2,115200,true, PCLK1_HZ, PCLK2_HZ );
	//Create ISIX blinking task
	isix_task_create( blinking_task, NULL, ISIX_PORT_SCHED_MIN_STACK_DEPTH, BLINKING_TASK_PRIO);
	//Create USB CDC class task
	isix_task_create( cdc_task, NULL, 1024, BLINKING_TASK_PRIO );
	dbprintf("Hello from USB+TCP sample");
	//Initialize the tcpip library
	tcp_eth_init();
	tcpecho_init();
	/* Initialize the usb serial */
	stm32_usbdev_serial_open();
	//Start the isix scheduler
	isix_start_scheduler();
}
/* ------------------------------------------------------------------ */
