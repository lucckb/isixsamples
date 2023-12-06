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
#include <stm32_ll_gpio.h>
#include <stm32_ll_fsmc.h>
#include <stm32_ll_rcc.h>
#include <foundation/sys/dbglog.h>


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
	LL_GPIO_InitTypeDef port_cnf {
		.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1|LL_GPIO_PIN_4|LL_GPIO_PIN_5|
			LL_GPIO_PIN_7|LL_GPIO_PIN_14|LL_GPIO_PIN_15,
		.Mode = LL_GPIO_MODE_ALTERNATE,
		.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH,
		.OutputType = LL_GPIO_OUTPUT_PUSHPULL,
		.Pull = LL_GPIO_PULL_NO,
		.Alternate = LL_GPIO_AF_12
	};
	// PORTD
	LL_GPIO_Init(GPIOD, &port_cnf);
	// PORTE
	port_cnf.Pin = LL_GPIO_PIN_7|LL_GPIO_PIN_8|LL_GPIO_PIN_9|LL_GPIO_PIN_10;
	LL_GPIO_Init(GPIOE, &port_cnf);
	// PORTF
	port_cnf.Pin = LL_GPIO_PIN_2;
	LL_GPIO_Init(GPIOF, &port_cnf);

}

void ili_fsmc_bus::fsmc_setup()
{
// TODO: FSMC bus do it later
#if 0
	FSMC_NORSRAM_TimingTypeDef rd_cfg {
		.AddressSetupTime = 1,
		.AddressHoldTime = 0,
		.DataSetupTime = 8,
		.BusTurnAroundDuration = 1,
		.CLKDivision = 0,
		.DataLatency = 0,
		.AccessMode = FSMC_ACCESS_MODE_A
	};
	FSMC_NORSRAM_TimingTypeDef wr_cfg {
		.AddressSetupTime = 1,
		.AddressHoldTime = 0,
		.DataSetupTime = 3,
		.BusTurnAroundDuration = 1,
		.CLKDivision = 0,
		.DataLatency = 0,
		.AccessMode = FSMC_ACCESS_MODE_A
	};
	FSMC_NORSRAM_InitTypeDef cfg {
		.NSBank = FSMC_NORSRAM_BANK1,
		.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE,
		.MemoryType = FSMC_MEMORY_TYPE_SRAM,
		.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_8,
		.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE,
		.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW,
		.WrapMode = FSMC_WRAP_MODE_DISABLE,
		.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS,
		.WriteOperation = FSMC_WRITE_OPERATION_ENABLE,
		.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE,
		.ExtendedMode = FSMC_EXTENDED_MODE_ENABLE,
		.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE,
		.WriteBurst = FSMC_WRITE_BURST_DISABLE,
		.ContinuousClock = FSMC_CONTINUOUS_CLOCK_SYNC_ONLY,
		.WriteFifo = 0,
		.PageSize = FSMC_PAGE_SIZE_NONE
	};
	FSMC_NORSRAM_Init(FSMC_NORSRAM_DEVICE, &cfg);
	FSMC_NORSRAM_Timing_Init(FSMC_NORSRAM_DEVICE, &rd_cfg, cfg.NSBank);
	FSMC_NORSRAM_Extended_Timing_Init(FSMC_NORSRAM_EXTENDED_DEVICE, &wr_cfg, cfg.NSBank, cfg.ExtendedMode);
#endif
}

//Constructor
ili_fsmc_bus::ili_fsmc_bus()
{
	//Normal GPIOS
	LL_GPIO_InitTypeDef port_cnf {
		.Pin = bv(LCD_BL_PIN),
		.Mode = LL_GPIO_MODE_OUTPUT,
		.Speed = LL_GPIO_SPEED_FREQ_LOW,
		.OutputType = LL_GPIO_OUTPUT_PUSHPULL,
		.Pull = LL_GPIO_PULL_NO,
		.Alternate = 0
	};
	LL_GPIO_Init(LCD_BL_PORT, &port_cnf);
	LL_GPIO_SetOutputPin( LCD_BL_PORT, bv(LCD_BL_PIN));
	port_cnf.Pin = bv(LCD_RST_PIN);
	LL_GPIO_Init(LCD_RST_PORT, &port_cnf);
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
		if( val ) LL_GPIO_SetOutputPin( LCD_RST_PORT, bv(LCD_RST_PIN));
		else LL_GPIO_ResetOutputPin( LCD_RST_PORT, bv(LCD_RST_PIN));
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
		LL_GPIO_SetOutputPin( LCD_BL_PORT, bv(LCD_BL_PIN) );
	} else {
		LL_GPIO_ResetOutputPin( LCD_BL_PORT, bv(LCD_BL_PIN));
	}
}

}

