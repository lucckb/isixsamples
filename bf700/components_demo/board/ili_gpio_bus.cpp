/*
 * ili_gpio_bus.cpp
 *
 *  Created on: 21 lis 2013
 *      Author: lucck
 */

#include <board/ili_gpio_bus.hpp>
#include <stm32_ll_gpio.h>
#include <stm32_ll_tim.h>
#include <isix.h>

namespace drv {

namespace {
	inline void gpio_set_clr_mask(GPIO_TypeDef* port , uint16_t enflags, uint16_t mask)
	{
		port->BSRR = (uint32_t)(enflags & mask) | ((uint32_t)( ~enflags & mask)<<16);
	}
	uint8_t data_get()
	{
		auto a1 = LL_GPIO_ReadInputPort(GPIOD) &  0xc003;
		auto n1 = (( a1>>14 ) & 0b0011 ) | ((a1<<2) & 0b1100 );
		auto n2 =  (LL_GPIO_ReadInputPort(GPIOE)&0x780) >> 7;
		return (n2<<4) |  n1;
	}

	void data_set( uint16_t val )
	{
		gpio_set_clr_mask( GPIOD, ((val&0b0011)<<14)|((val&0b1100)>>2), 0xc003 );
		gpio_set_clr_mask( GPIOE, ((val&0xf0)>>4)<<7, 0x780 );
	}
}


void ili_gpio_bus::bus_dir( ili_gpio_bus::dir_t dir )
{
	if( dir==dir_t::out && m_dir==dir_t::in )
	{
		LL_GPIO_InitTypeDef port_cnf {
			.Pin = 0x780,
			.Mode = LL_GPIO_MODE_OUTPUT,
			.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH,
			.OutputType = LL_GPIO_OUTPUT_PUSHPULL,
			.Pull = LL_GPIO_PULL_NO,
			.Alternate = 0
		};
		LL_GPIO_Init(GPIOE, &port_cnf);
		port_cnf.Pin = 0xc003;
		LL_GPIO_Init(GPIOD, &port_cnf);
	}
	else if( dir==dir_t::in && m_dir==dir_t::out  )
	{
		LL_GPIO_InitTypeDef port_cnf {
			.Pin = 0x780,
			.Mode = LL_GPIO_MODE_INPUT,
			.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH,
			.OutputType = LL_GPIO_OUTPUT_PUSHPULL,
			.Pull = LL_GPIO_PULL_UP,
			.Alternate = 0
		};
		LL_GPIO_Init(GPIOE, &port_cnf);
		port_cnf.Pin = 0xc003;
		LL_GPIO_Init(GPIOD, &port_cnf);
	}
	m_dir = dir;
}

//Constructor
ili_gpio_bus::ili_gpio_bus()
{
	//Enable periph backlight
	LL_GPIO_InitTypeDef port_cnf {
		.Pin = 0x780,
		.Mode = LL_GPIO_MODE_INPUT,
		.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH,
		.OutputType = LL_GPIO_OUTPUT_PUSHPULL,
		.Pull = LL_GPIO_PULL_UP,
		.Alternate = 0
	};
	LL_GPIO_Init(GPIOE, &port_cnf);
	port_cnf.Pin = 0xc003;
	LL_GPIO_Init(GPIOD, &port_cnf);

	port_cnf.Pin = bv(7)|bv(5)|bv(4);
	port_cnf.Mode = LL_GPIO_MODE_OUTPUT;
	port_cnf.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOD, &port_cnf);

	port_cnf.Pin = bv(2)|bv(3);
	LL_GPIO_Init(GPIOF, &port_cnf);
	LL_GPIO_SetOutputPin(GPIOD, bv(7)|bv(5)|bv(4));
	LL_GPIO_SetOutputPin(GPIOF, bv(2)|bv(3)); /* PWM conf*/

	port_cnf.Pin = bv(6);
	port_cnf.Speed = LL_GPIO_SPEED_FREQ_LOW;
	LL_GPIO_Init(GPIOE, &port_cnf);
	LL_GPIO_SetOutputPin(GPIOE, bv(6));
}


// Lock bus and set address
void ili_gpio_bus::set_ctlbits( int bit, bool val )
{
	const auto io_fn = val?(&LL_GPIO_SetOutputPin):(&LL_GPIO_ResetOutputPin);
	switch( bit )
	{
	case CSL_BIT_CMD: io_fn( GPIOD, bv(7) ); break;
	case RS_BIT_CMD:  io_fn( GPIOF, bv(2) ); break;
	case RST_BIT_CMD: io_fn( GPIOF, bv(3) ); break;
	}
}

/* Read transfer */
void ili_gpio_bus::read( void *buf, std::size_t len )
{
	bus_dir( dir_t::in );
	for( size_t l=0; l<len; ++l )
	{
		LL_GPIO_ResetOutputPin( GPIOD, bv(4) );
		asm volatile(
			"nop\t\n"
			"nop\t\n"
			"nop\t\n"
			"nop\t\n"
		);
		*(reinterpret_cast<uint8_t*>(buf)+l) = data_get();
		LL_GPIO_SetOutputPin( GPIOD, bv(4) );
	}
}

/* Write transfer */
void ili_gpio_bus::write( const void *buf, size_t len )
{
	bus_dir( dir_t::out );
	for( size_t l=0; l<len; ++l )
	{
		//One write
		data_set( *(reinterpret_cast<const uint8_t*>(buf)+l) );
		LL_GPIO_ResetOutputPin( GPIOD, bv(5) );	//WR down
		LL_GPIO_SetOutputPin( GPIOD, bv(5) );	//WR up
	}
}

/* Fill pattern */
void ili_gpio_bus::fill( unsigned value, size_t nelms )
{
	bus_dir( dir_t::out );
	for( size_t l=0; l<nelms; ++l )
	{
		//One write
		data_set( value );
		LL_GPIO_ResetOutputPin( GPIOD, bv(5) );	//WR down
		LL_GPIO_SetOutputPin( GPIOD, bv(5) );	//WR up
		data_set( value>>8 );
		LL_GPIO_ResetOutputPin( GPIOD, bv(5) );	//WR down
		LL_GPIO_SetOutputPin( GPIOD, bv(5) );	//WR up
	}
}

/* Set PWM  */
void ili_gpio_bus::set_pwm( int percent )
{
	static constexpr auto c_scale1 = 1000;
	static constexpr auto c_scale2 = 550;
	if( percent > 100 ) percent = 100;
	else if( percent < 0 ) percent = 0;
	percent = (percent*c_scale2)/c_scale1;
	LL_TIM_OC_SetCompareCH3(TIM3, percent);
}

/* Wait ms long delay */
void ili_gpio_bus::delay( unsigned tout )
{
	isix_wait_ms( tout );
}

}

