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
		0x00,		//Address
		0x00,		//Sekundy
		0x34,		//Minuty
		0x14,		//Godziny
		0x01,		//Dzien tyg
		0x12,		//Dzien
		0x03,		//Miesiac
		0x07		//Rok
	};

	i2c_bus.i2c_write_7bit( I2C_RTC_ADDR, pgm_regs, sizeof(pgm_regs) );

	for(;;)
	{
		isix::isix_wait(10000);
	}
}

/* ------------------------------------------------------------------ */

}
