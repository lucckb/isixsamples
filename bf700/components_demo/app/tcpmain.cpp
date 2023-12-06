/*
 * =====================================================================================
 *
 *       Filename:  tcpmain.cpp
 *
 *    Description:  TCP demo main
 *
 *        Version:  1.0
 *        Created:  30.08.2016 17:25:56
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Lucjan Bryndza (LB), lbryndza.p(at)boff(dot)pl
 *   Organization:  BoFF
 *
 * =====================================================================================
 */

#include <isix.h>
#include <foundation/sys/dbglog.h>
#include <config/conf.h>
#include <lwip/tcpip.h>
#include <lwip/dhcp.h>
#include <eth/ethernetif.h>
#include "tcpecho/tcpecho.h"


namespace app {

namespace {

/**
  * @brief  Initializes the lwIP stack
  * @param  None
  * @retval None
  *
  */
void netif_callback( struct netif *netif )
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


void tcp_eth_init(void)
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
	  dbg_err("Unable to create network interface");
	  abort();
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

}	//Unnamed namespace end

namespace tcp {
		void init()
		{
			tcp_eth_init();
			tcpecho_init();
			dbg_info("TCPIP stack initialized\n");
		}

}

}

