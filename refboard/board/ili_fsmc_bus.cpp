/*
 * =====================================================================================
 *
 *       Filename:  ili_fsmc_bus.cpp
 *
 *    Description:  ILI display fsmc bus
 *
 *        Version:  1.0
 *        Created:  27.08.2016 14:14:12
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Lucjan Bryndza (LB), lbryndza.p(at)boff(dot)pl
 *   Organization:  BoFF
 *
 * =====================================================================================
 */
#include <board/ili_fsmc_bus.hpp>
#include <isix.h>
#include <stm32gpio.h>
#include <stm32fsmc.h>
#include <stm32rcc.h>
#include <foundation/dbglog.h>


/** TODO: DMA transfer support 
 *  use specialized fsmc driver from stdlib */


/* 
  PD0   ------> FSMC_D2
  PD1   ------> FSMC_D3
  PD4   ------> FSMC_NOE
  PD5   ------> FSMC_NWE
  PD7   ------> FSMC_NE1
  PD14  ------> FSMC_D0
  PD15  ------> FSMC_D1

  PE7   ------> FSMC_D4
  PE8   ------> FSMC_D5
  PE9   ------> FSMC_D6
  PE10  ------> FSMC_D7

  PF2   ------> FSMC_A2
   * */


namespace drv {

namespace {
	const auto LCD_BL_PORT = GPIOE;
	constexpr auto LCD_BL_PIN = 6;
	const auto LCD_RST_PORT = GPIOF;
	const auto LCD_RST_PIN = 3;
	constexpr auto FSMC_NOR_BANK1_BASE = 0x60000000;

}

void ili_fsmc_bus::fsmc_gpio_setup()
{
	using namespace stm32;
	auto gpio_fsmc =
	[]( GPIO_TypeDef* port, uint16_t pin ) {
		using namespace stm32;
		gpio_config( port, pin, GPIO_MODE_ALTERNATE, GPIO_PUPD_PULLUP, GPIO_SPEED_100MHZ, GPIO_OTYPE_PP );
	};
	// PortD
	gpio_pin_AF_config( GPIOD, GPIO_PinSource0, GPIO_AF_FSMC ); gpio_fsmc( GPIOD, 0 );
	gpio_pin_AF_config( GPIOD, GPIO_PinSource1, GPIO_AF_FSMC ); gpio_fsmc( GPIOD, 1 );
	gpio_pin_AF_config( GPIOD, GPIO_PinSource4, GPIO_AF_FSMC );	gpio_fsmc( GPIOD, 4 );
	gpio_pin_AF_config( GPIOD, GPIO_PinSource5, GPIO_AF_FSMC ); gpio_fsmc( GPIOD, 5 );
	gpio_pin_AF_config( GPIOD, GPIO_PinSource7, GPIO_AF_FSMC ); gpio_fsmc( GPIOD, 7 );
	gpio_pin_AF_config( GPIOD, GPIO_PinSource14, GPIO_AF_FSMC ); gpio_fsmc( GPIOD, 14 );
	gpio_pin_AF_config( GPIOD, GPIO_PinSource15, GPIO_AF_FSMC ); gpio_fsmc( GPIOD, 15 );
	// PortE
	gpio_pin_AF_config( GPIOE, GPIO_PinSource7, GPIO_AF_FSMC ); gpio_fsmc( GPIOE, 7 );
	gpio_pin_AF_config( GPIOE, GPIO_PinSource8, GPIO_AF_FSMC ); gpio_fsmc( GPIOE, 8 );
	gpio_pin_AF_config( GPIOE, GPIO_PinSource9, GPIO_AF_FSMC ); gpio_fsmc( GPIOE, 9 );
	gpio_pin_AF_config( GPIOE, GPIO_PinSource10, GPIO_AF_FSMC ); gpio_fsmc( GPIOE, 10 );
	// PortF
	gpio_pin_AF_config( GPIOF, GPIO_PinSource2, GPIO_AF_FSMC ); gpio_fsmc( GPIOF, 2 );

}

void ili_fsmc_bus::fsmc_setup()
{
	using namespace stm32;
	const fsmc_timing rd_cfg {
		.address_setup_time = 1,
		.address_hold_time = 0,
		.data_setup_time = 8,
		.bus_turn_arround_duration = 1,
		.clk_div = 0,
		.data_latency = 0,
		.access_mode = FSMC_AccessMode_A
	};
	const fsmc_timing wr_cfg {
		.address_setup_time = 1,
		.address_hold_time = 0,
		.data_setup_time = 3,
		.bus_turn_arround_duration = 1,
		.clk_div = 0,
		.data_latency = 0,
		.access_mode = FSMC_AccessMode_A
	};
	rcc_ahb3_periph_clock_cmd( RCC_AHB3Periph_FSMC, true );
	fsmc_nor_setup( FSMC_Bank1_NORSRAM1,
			FSMC_DataAddressMux_Disable | FSMC_MemoryType_SRAM |
			FSMC_MemoryDataWidth_8b | FSMC_BurstAccessMode_Disable |
			FSMC_WaitSignalPolarity_Low |  FSMC_WrapMode_Disable |
			FSMC_WaitSignalActive_BeforeWaitState | FSMC_WriteOperation_Enable |
			FSMC_WaitSignal_Disable | FSMC_ExtendedMode_Enable |
			FSMC_AsynchronousWait_Disable | FSMC_WriteBurst_Disable,
			&rd_cfg, &wr_cfg );
	fsmc_nor_cmd( FSMC_Bank1_NORSRAM1, true );
}

//Constructor
ili_fsmc_bus::ili_fsmc_bus()
{
	using namespace stm32;
	//Normal GPIOS
	gpio_config( LCD_BL_PORT, LCD_BL_PIN, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_SPEED_2MHZ );
	gpio_set( LCD_BL_PORT, LCD_BL_PIN );
	gpio_config( LCD_RST_PORT, LCD_RST_PIN, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_SPEED_2MHZ );
	fsmc_gpio_setup();
	fsmc_setup();
}

// Lock bus and set address
void ili_fsmc_bus::set_ctlbits( int bit, bool val )
{
	switch( bit ) {
	//Do nothing (automatic in the bus mode)
	case CSL_BIT_CMD:
		break;
	case RS_BIT_CMD:
		if( val ) m_bus_addr = reinterpret_cast<volatile uint8_t*>( FSMC_NOR_BANK1_BASE+4);
		else m_bus_addr = reinterpret_cast<volatile uint8_t*>( FSMC_NOR_BANK1_BASE );
		break;
	case RST_BIT_CMD:
		if( val ) stm32::gpio_set( LCD_RST_PORT, LCD_RST_PIN );
		else stm32::gpio_set( LCD_RST_PORT, LCD_RST_PIN );
		break;
	}
}

/* Read transfer */
void ili_fsmc_bus::read( void *buf, std::size_t len )
{
	for( size_t p=0; p<len; ++p ) {
		reinterpret_cast<uint8_t*>(buf)[p] = *m_bus_addr;
	}
}
/* Write transfer */
void ili_fsmc_bus::write( const void *buf, size_t len )
{
	for( size_t p=0; p<len; ++p ) {
		*m_bus_addr = reinterpret_cast<const uint8_t*>(buf)[p];
	}
}
/* Fill pattern */
void ili_fsmc_bus::fill( unsigned value, size_t nelms )
{
	for( size_t p=0; p<nelms; ++p ) {
		*m_bus_addr = value;
		*m_bus_addr = value>>8;
	}
}
/* Wait ms long delay */
void ili_fsmc_bus::delay( unsigned tout )
{
	isix::wait_ms( tout );
}
/* Set PWM  */
void ili_fsmc_bus::set_pwm( int percent )
{
	if( percent > 0 ) {
		stm32::gpio_set( LCD_BL_PORT, LCD_BL_PIN );
	} else {
		stm32::gpio_clr( LCD_BL_PORT, LCD_BL_PIN );
	}
}

}

