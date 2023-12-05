/*
 * =====================================================================================
 *
 *       Filename:  codec_test.cpp
 *
 *    Description:  STM32 codec test
 *
 *        Version:  1.0
 *        Created:  02.09.2016 19:29:22
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Lucjan Bryndza (LB), lbryndza.p(at)boff(dot)pl
 *   Organization:  BoFF
 *
 * =====================================================================================
 */

#include <cstdint>
#include <foundation/sys/dbglog.h>
#include <periph/drivers/i2c/i2c_master.hpp>
#include <isix.h>
#include <isix/arch/irq.h>
#include <stm32_ll_rcc.h>
#include <stm32_ll_spi.h>
#include <stm32_ll_dma.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_system.h>
#include <stm32_ll_gpio.h>
#include <cmath>


// FIXME: Conflict on DMA stream 0 and 5 with I2C1_RX
//   Update STM32 driver without DMA on the interrupt mode only

namespace app {

namespace {

#define I2S3_DR_Address						(&SPI3->DR)
#define I2S3ext_DR_Address					(&I2S3ext->DR)

#define DMA1_S3_PRIORITY					0
#define DMA1_S3_SUBPRIORITY					0



#define I2S3_SDTI_PORT						GPIOB
#define I2S3_SDTI_PIN						4

#define I2S3_SDTO_PORT						GPIOB
#define I2S3_SDTO_PIN						5

#define I2S3_BICK_PORT						GPIOB
#define I2S3_BICK_PIN						3

#define I2S3_LRCK_PORT						GPIOA
#define I2S3_LRCK_PIN						15

#define I2S3_MCLK_PORT						GPIOC
#define I2S3_MCLK_PIN						7

#define CHANNEL_BUFFER_SIZE				128
#define BUFFER_SIZE							2*CHANNEL_BUFFER_SIZE


	static signed long buf1[BUFFER_SIZE];
	static signed long buf2[BUFFER_SIZE];

	constexpr auto wm8731_addr_0 = 0x1A;
	constexpr auto codec_addr = wm8731_addr_0<<1;
	constexpr uint16_t w8731_init_data[] = 
	{
		0b000011111,			// Reg 00: Left Line In (0dB, mute off)
		0b000011111,			// Reg 01: Right Line In (0dB, mute off)
		0x079,			// Reg 02: Left Headphone out (0dB)
		0x079,			// Reg 03: Right Headphone out (0dB)
		0b000011101,			// Reg 04: Analog Audio Path Control (DAC sel, Mute Mic)
		0x000,			// Reg 05: Digital Audio Path Control
		0b000000000,			// Reg 06: Power Down Control (Clkout, Osc, Mic Off)
		//	0x00E,			// Reg 07: Digital Audio Interface Format (i2s, 32-bit, slave)
		0b000001000,			// Reg 07: Digital Audio Interface Format (i2s, 16-bit, slave)
		0x000,			// Reg 08: Sampling Control (Normal, 256x, 48k ADC/DAC)
		0x001			// Reg 09: Active Control
	};

	int codec_write_register(periph::drivers::i2c_master& bus, uint8_t addr, uint16_t val)
	{
		/* Assemble 2-byte data in WM8731 format */
		const uint8_t arr[2] {
			uint8_t(((addr<<1)&0xFE) | ((val>>8)&0x01)),
				uint8_t(val&0xFF)
		};
		periph::blk::tx_transfer tran(arr, sizeof arr);
		return bus.transaction(codec_addr, tran);
	}
}


int codec_reset(periph::drivers::i2c_master& bus)
{
	auto ret = codec_write_register( bus, 0x0f, 0 );
	if( ret ) {
		dbg_err( "CODEC unable to write codec data %i", ret );
		return ret;
	}
	//Get some time for device reset
	isix::wait_ms(100);
	for( size_t reg = 0; reg<sizeof(w8731_init_data)/sizeof(w8731_init_data[0]); ++reg ) {
		if ( (ret=codec_write_register(bus,reg,w8731_init_data[reg])) ) {
			dbg_err("CODEC wr reg %lu err %i", reg, ret );
		}
	}
	return ret;
}

void i2s_init()
{
	//I2S PLL Config
	LL_RCC_SetI2SClockSource(LL_RCC_I2S1_CLKSOURCE_PLLI2S);
	//TODO check me exactly
	LL_RCC_PLLI2S_ConfigDomain_I2S(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLI2SM_DIV_2, 192, LL_RCC_PLLI2SR_DIV_2);
	LL_RCC_PLLI2S_Enable();
	while(!LL_RCC_PLLI2S_IsReady());

	//DMA1 Clock Enable
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

	//SPI2 Clock Enable
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);
	dbg_info("PLL is ok\n");

	//AF Config
	LL_GPIO_InitTypeDef io_cfg {
		.Pin = LL_GPIO_PIN_0,
		.Mode = LL_GPIO_MODE_ALTERNATE,
		.Speed = LL_GPIO_SPEED_FREQ_HIGH,
		.OutputType = LL_GPIO_OUTPUT_PUSHPULL,
		.Pull = LL_GPIO_PULL_NO,
		.Alternate = LL_GPIO_AF_7
	};
	io_cfg.Pin = 1U << I2S3_SDTI_PIN;
	LL_GPIO_Init(I2S3_SDTI_PORT, &io_cfg); // SDTI1
	io_cfg.Alternate = LL_GPIO_AF_6;
	io_cfg.Pin = 1U << I2S3_SDTO_PIN;
	LL_GPIO_Init(I2S3_SDTO_PORT, &io_cfg); // SDTO1
	io_cfg.Pin = 1U << I2S3_BICK_PIN;
	LL_GPIO_Init(I2S3_BICK_PORT, &io_cfg); // BICK
	io_cfg.Pin = 1U << I2S3_LRCK_PIN;
	LL_GPIO_Init(I2S3_LRCK_PORT, &io_cfg); // LRCK1
	io_cfg.Pin = 1U << I2S3_MCLK_PIN;
	LL_GPIO_Init(I2S3_MCLK_PORT, &io_cfg); // LRCK1
	
	//NVIC Config
	isix::set_irq_priority(DMA1_Stream0_IRQn, {DMA1_S3_PRIORITY, DMA1_S3_SUBPRIORITY});
	isix::request_irq(DMA1_Stream0_IRQn);

	//DMA1 Config (Circural Mode, Double Buffering, Packing)
	{	
		// RX
		LL_DMA_InitTypeDef dma_cfg {
			.PeriphOrM2MSrcAddress = (uintptr_t)I2S3ext_DR_Address,
			.MemoryOrM2MDstAddress = (uintptr_t)&buf1,
			.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY,
			.Mode = LL_DMA_MODE_CIRCULAR,
			.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT,
			.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT,
			.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_HALFWORD,
			.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_WORD,
			.NbData = 2 * BUFFER_SIZE,
			.Channel = LL_DMA_CHANNEL_3,
			.Priority = LL_DMA_PRIORITY_HIGH,
			.FIFOMode = LL_DMA_FIFOMODE_DISABLE,
			.FIFOThreshold = LL_DMA_FIFOTHRESHOLD_1_2,
			.MemBurst = LL_DMA_MBURST_INC4,
			.PeriphBurst = LL_DMA_PBURST_INC4
		};
		LL_DMA_Init(DMA1, LL_DMA_STREAM_0, &dma_cfg); 
	}
	{	
		// TX
		LL_DMA_InitTypeDef dma_cfg {
			.PeriphOrM2MSrcAddress = (uintptr_t)I2S3_DR_Address,
			.MemoryOrM2MDstAddress = (uintptr_t)&buf1,
			.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH,
			.Mode = LL_DMA_MODE_CIRCULAR,
			.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT,
			.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT,
			.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_HALFWORD,
			.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_WORD,
			.NbData = 2 * BUFFER_SIZE,
			.Channel = LL_DMA_CHANNEL_0,
			.Priority = LL_DMA_PRIORITY_HIGH,
			.FIFOMode = LL_DMA_FIFOMODE_DISABLE,
			.FIFOThreshold = LL_DMA_FIFOTHRESHOLD_1_2,
			.MemBurst = LL_DMA_MBURST_INC4,
			.PeriphBurst = LL_DMA_PBURST_INC4
		};
		LL_DMA_Init(DMA1, LL_DMA_STREAM_5, &dma_cfg); 
	}

	LL_DMA_SetIncOffsetSize(DMA1, LL_DMA_STREAM_0, LL_DMA_OFFSETSIZE_PSIZE);
	LL_DMA_SetIncOffsetSize(DMA1, LL_DMA_STREAM_5, LL_DMA_OFFSETSIZE_PSIZE);

	LL_DMA_SetMemory1Address(DMA1, LL_DMA_STREAM_0, (uintptr_t)buf2);
	LL_DMA_SetMemory1Address(DMA1, LL_DMA_STREAM_5, (uintptr_t)buf2);

	LL_DMA_EnableDoubleBufferMode(DMA1, LL_DMA_STREAM_0);
	LL_DMA_EnableDoubleBufferMode(DMA1, LL_DMA_STREAM_5);

	LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_0);

	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_0);
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_5);

	//I2S2 Config
	LL_I2S_InitTypeDef spi_cfg {
		.Mode = LL_I2S_MODE_MASTER_TX,
		.Standard = LL_I2S_STANDARD_PHILIPS,
		.DataFormat = LL_I2S_DATAFORMAT_24B,
		.MCLKOutput = LL_I2S_MCLK_OUTPUT_ENABLE,
		.AudioFreq = LL_I2S_AUDIOFREQ_48K,
		.ClockPolarity = LL_I2S_POLARITY_LOW
	};
	LL_I2S_Init(SPI3, &spi_cfg);
	LL_I2S_InitFullDuplex(I2S3ext, &spi_cfg);
	LL_I2S_EnableDMAReq_TX(SPI3);
	LL_I2S_EnableDMAReq_RX(I2S3ext);

	LL_I2S_Enable(I2S2ext);
	LL_I2S_Enable(SPI3);
}


namespace {
	//Global buffer for float ops
	float fBuf[BUFFER_SIZE];
	float phase_acc = 0;
	constexpr auto sin_size = 64U;
	float sintable[ sin_size ];
	constexpr float FREQ = 1000;
	constexpr float SAMPLE_RATE = 48000;
	constexpr float PHI =  FREQ / SAMPLE_RATE * sin_size;
	constexpr float AMPL = 1<<16;

}

#if 1
// FIXME this interrupt
// We need to merge with real dma driver
extern "C"
void __attribute__ ((optimize(3), optimize("unroll-loops"))) dma1_stream0_isr_vector_(void)
{
	if(LL_DMA_IsActiveFlag_TC0(DMA1))
	{
		signed long *slBuf1, *slBuf2;
		slBuf1 = slBuf2 = (LL_DMA_GetCurrentTargetMem(DMA1, LL_DMA_STREAM_0)) ? buf1 : buf2;

		for(unsigned long i = 0; i < BUFFER_SIZE; i++)
		{
			signed long tmp = *slBuf1++;

			asm volatile
			(
				"ROR %[result], %[data], #16 \n"
				: [result]"=&r"(tmp)
				: [data]"r"(tmp)
			);

			fBuf[i] = tmp;
		}

		//DSP_Exec2Channel(fBuf, CHANNEL_BUFFER_SIZE);
		for( int i=0; i< BUFFER_SIZE; i++ ) {
				fBuf[i] =  fBuf[i] * 3 + AMPL * sintable[ int(phase_acc) ];
				if( 1 ) {
					phase_acc += PHI;
					if( phase_acc >= sin_size ) {
						phase_acc -= sin_size;
					}
				}
		}

		//F32 to S32 Convert
		for(unsigned long i = 0; i < BUFFER_SIZE; i++)
		{
			signed long tmp = fBuf[i];

			asm volatile
			(
				"ROR %[result], %[data], #16 \n"
				: [result]"=&r"(tmp)
				: [data]"r"(tmp)
			);

			*slBuf2++ = tmp;
		}
		LL_DMA_ClearFlag_TC0(DMA1);
	} else 	if( LL_DMA_IsActiveFlag_TE0(DMA1)) {
		dbg_err("IS error abort");
		abort();

	} else {
		dbg_err("Inny %08x %08x", DMA1->HISR, DMA1->LISR );
		abort();
	}
}
#endif


void codec_task( periph::drivers::i2c_master& bus )
{
	//Generate sine table
	for( size_t i=0; i<sin_size; ++i) {
		sintable[i] = std::sin( 2.0f * M_PI * float(i) / float(sin_size) ) + 1.0f;
	}
	//Test codec reprogram 100 times
	//for( int i=0; i<100; ++i ) {
	if(1) {
		if( codec_reset( bus ) ) {
			dbg_err("Codec Reset failed I2S config skipped");
			isix_wait_ms(100);
			return;
		}
		isix_wait_ms(10);
	}
	i2s_init();
}


}
