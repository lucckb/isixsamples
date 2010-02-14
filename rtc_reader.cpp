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
/* ------------------------------------------------------------------ */
namespace app
{

/* ------------------------------------------------------------------ */
namespace
{
	const uint8_t I2C_RTC_ADDR = 208;
}

/* ------------------------------------------------------------------ */
rtc_reader::rtc_reader(): task_base(STACK_SIZE, TASK_PRIO), i2c_bus(I2C1)
{
	// TODO Auto-generated constructor stub

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
	static uint8_t buf[12];
	int status;
	i2c_bus.i2c_transfer_7bit(I2C_RTC_ADDR,pgm_regs,sizeof(pgm_regs),NULL,0);
	i2c_bus.i2c_transfer_7bit(I2C_RTC_ADDR,&sw_addr,sizeof(sw_addr),buf,8);

	i2c_bus.i2c_transfer_7bit(I2C_RTC_ADDR,&sw_addr,sizeof(sw_addr),buf,8);
	for(;;)
	{
		status = i2c_bus.i2c_transfer_7bit(I2C_RTC_ADDR,&sw_addr,sizeof(sw_addr),buf,3);
		if(status==isix::ISIX_EOK)
			tiny_printf("%02x:%02x:%02x\r\n",buf[2],buf[1],buf[0]);
		else
			tiny_printf("!ERROR=%d\r\n",status);
		isix::isix_wait(1300);
	}
}

/* ------------------------------------------------------------------ */

}
