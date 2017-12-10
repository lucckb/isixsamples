/* ------------------------------------------------------------------ */
/*
 * rtc_reader.cpp
 *
 *  Created on: 2010-02-07
 *      Author: lucck
 */
/* ------------------------------------------------------------------ */
#include "rtc_reader.hpp"
#include <stm32lib.h>
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
	    i2c_bus(I2C1), disp_srv(disp_srv_)
{

}
/* ------------------------------------------------------------------ */
//Main rtc reader core task
void rtc_reader::main() noexcept
{
	static const uint8_t pgm_regs[] =
	{
		0x01,		//Sec
		0x02,		//Min
		0x03,		//Hour
		0x04,		//Day num
		0x05,		//day
		0x06,		//Month
		0x07,		//Year
		0x00		//Config
	};

	//Software address
	static const uint8_t sw_addr = 0;
	static uint8_t buf[3];
	static time_msg tmsg( 3,2 );
	int status;
	//Send configuration registgers
	i2c_bus.i2c_transfer_7bit(I2C_RTC_ADDR,pgm_regs,sizeof(pgm_regs),NULL,0);
	//Main task loop
	for(;;)
	{
		//Send configuration registgers
		status = i2c_bus.i2c_transfer_7bit(I2C_RTC_ADDR,&sw_addr,sizeof(sw_addr),buf, sizeof(buf) );

		if(status>=0)
		{
			//If no error display hours
			tmsg.set_time( buf[2]&0x3f, buf[1]&0x7f, buf[0]&0x7f );
		}
		else
		{
			//If error display on screen
			tmsg.set_text("I2C ERR");
		}
		//Send message to the i2c device
		disp_srv.send_message(tmsg);
		//Refresh screen ratio
		isix::wait_ms(200);
	}
}

/* ------------------------------------------------------------------ */

}
