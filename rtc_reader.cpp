/* ------------------------------------------------------------------ */
/*
 * rtc_reader.cpp
 *
 *  Created on: 2010-02-07
 *      Author: lucck
 */
/* ------------------------------------------------------------------ */
#include "rtc_reader.hpp"
#include <stm32f10x_lib.h>
#include <tiny_printf.h>
#include "display_server.hpp"
#include <cstring>
/* ------------------------------------------------------------------ */
namespace app
{

/* ------------------------------------------------------------------ */
namespace
{
	const uint8_t I2C_RTC_ADDR = 208;
}

/* ------------------------------------------------------------------ */
rtc_reader::rtc_reader(display_server &disp_srv_):
		task_base(STACK_SIZE, TASK_PRIO),
	    i2c_bus(I2C1), disp_srv(disp_srv_)
{

}
/* ------------------------------------------------------------------ */
//Main rtc reader core task
void rtc_reader::main()
{
	static const uint8_t pgm_regs[] =
	{
		0x01,		//Sekundy
		0x02,		//Minuty
		0x03,		//Godziny
		0x04,		//Dzien tyg
		0x05,		//Dzien
		0x06,		//Miesiac
		0x07,		//Rok
		0x00		//Konfiguracja
	};

	static const uint8_t sw_addr = 0;
	static uint8_t buf[8];
	static time_msg tmsg( 3,2 );
	int status;

	i2c_bus.i2c_transfer_7bit(I2C_RTC_ADDR,pgm_regs,sizeof(pgm_regs),NULL,0);

	for(;;)
	{
		status = i2c_bus.i2c_transfer_7bit(I2C_RTC_ADDR,&sw_addr,sizeof(sw_addr),buf,2 );
		if(status==isix::ISIX_EOK)
		{
			tmsg.set_time( buf[2]&0x3f, buf[1]&0x7f, buf[0]&0x7f );
			tiny_printf("O%d\r\n",buf[0]);
		}
		else
		{
			tmsg.set_text("I2C ERR");
			tiny_printf("E%d\r\n",status);
		}
		disp_srv.send_message(tmsg);
		isix::isix_wait(1000);
	}
}

/* ------------------------------------------------------------------ */

}
