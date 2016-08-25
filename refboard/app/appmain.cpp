/*
 * =====================================================================================
 *
 *       Filename:  appmain.cpp
 *
 *    Description:  Application startup 
 *
 *        Version:  1.0
 *        Created:  25.08.2016 21:49:00
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Lucjan Bryndza (LB), lbryndza.p(at)boff(dot)pl
 *   Organization:  BoFF
 *
 * =====================================================================================
 */

#include <foundation/dbglog.h>
#include <usart_simple.h>
#include <config.h>

int main() {
	 dblog_init( stm32::usartsimple_putc, NULL, stm32::usartsimple_init,
	    		USART1,115200, false, CONFIG_PCLK1_HZ, CONFIG_PCLK2_HZ );
	 for(;;) {
		dbprintf(" SDIO test ");
		for( int i=0;i<10000000;++i) asm volatile("nop\n");
	}
	
	return 0;
}


