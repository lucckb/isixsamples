/*
 * =====================================================================================
 *
 *       Filename:  dac_audio.cpp
 *
 *    Description:  DAC Audio implementation
 *
 *        Version:  1.0
 *        Created:  27.02.2014 20:45:22
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Lucjan Bryndza (LB), lucck(at)boff(dot)pl
 *   Organization:  BoFF
 *
 * =====================================================================================
 */

#include <config/conf.h>
#include <cstdlib>
#include <new>
#include <stm32_ll_rcc.h>
#include <stm32_ll_tim.h>
#include <stm32_ll_gpio.h>
#include <stm32_ll_dac.h>
#include <stm32_ll_dma.h>
#include <stm32_ll_bus.h>
#include <isix/arch/irq.h>
#include <foundation/sys/dbglog.h>
#include "dac_audio.hpp"

namespace drv {

namespace {
	inline void terminate() {
#ifdef __EXCEPTIONS
		throw std::bad_alloc();
#else
		std::abort();
#endif
	}
	//!Configuration DATA
	static const auto DAC_GPIO_PORT = GPIOA;
	static const auto AUDIO_TIMER = TIM6;
	static constexpr auto AUDIO_TIMER_RCC =  LL_APB1_GRP1_PERIPH_TIM6;
	static constexpr auto DAC_DMA_RCC = LL_AHB1_GRP1_PERIPH_DMA1;
	static constexpr auto DAC_TRIGER = LL_DAC_TRIG_EXT_TIM6_TRGO;
	static constexpr auto DAC_CHANNEL = LL_DAC_CHANNEL_1;
	static const auto DAC_DMA_STREAM = LL_DMA_STREAM_5;
	static constexpr auto DAC_DMA_CHANNEL = LL_DMA_CHANNEL_7;
	static constexpr auto DAC_DMA_STREAM_IRQN = DMA1_Stream5_IRQn;
	// Pointer to the dac audio for interrupt
	dac_audio* g_dac_audio;
}

//! Constructor
dac_audio::dac_audio( size_t audio_buf_len, size_t num_buffers )
	: m_mempool( isix::mempool_create(
			num_buffers+NR_EXTRA_BUFS, sizeof(uint16_t)*audio_buf_len) ),
	  m_fifo( num_buffers ), m_done_sem(0, 1), m_mempool_len( audio_buf_len )
{
	//!If gdac audio
	if( g_dac_audio ) {
		terminate();
	}
	//!If unable to allocate required mempool segments
	if( m_mempool == nullptr ) {
		terminate();
	}
	if( !m_fifo.is_valid() ) {
		terminate();
	}
	m_error = hardware_setup();
	if( m_error ) {
		terminate();
	}
	g_dac_audio = this;
}

//! Destructor
dac_audio::~dac_audio()
{
	do_stop();
	isix::free_irq( DAC_DMA_STREAM_IRQN);
	LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_DAC1|AUDIO_TIMER_RCC);
	flush_buffers();
	isix::mempool_destroy( m_mempool );
	g_dac_audio = nullptr;
}

//! Setup the hardware
int dac_audio::hardware_setup()
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA|DAC_DMA_RCC);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_DAC1|AUDIO_TIMER_RCC);
	/* Configure GPIO */
    LL_GPIO_InitTypeDef dac_out {
        .Pin = LL_GPIO_PIN_4,
        .Mode =  LL_GPIO_MODE_ANALOG,
        .Speed = 0,
        .OutputType = 0,
        .Pull = LL_GPIO_PULL_NO,
        .Alternate = 0
    };
    LL_GPIO_Init( GPIOA , &dac_out);

	/* Configure DAC converter */
    LL_DAC_InitTypeDef dac = {
        .TriggerSource = DAC_TRIGER,
        .WaveAutoGeneration = LL_DAC_WAVE_AUTO_GENERATION_NONE,
        .WaveAutoGenerationConfig = 0,
        .OutputBuffer = LL_DAC_OUTPUT_BUFFER_ENABLE
    };
	LL_DAC_Init( DAC1,DAC_CHANNEL ,&dac);
    isix::set_irq_priority( DAC_DMA_STREAM_IRQN, {IRQ_PRIO, IRQ_SUB} );
    isix::request_irq( DAC_DMA_STREAM_IRQN );
	return err_success;
}

int dac_audio::play( unsigned short fs )
{
	if( m_state != state::idle ) {
		dbprintf("Already playing");
		m_error = err_playing;
		return m_error;
	}
	//Configure the timer
	static const unsigned val =  CONFIG_PCLK1_HZ / (fs / 2);
	if( CONFIG_PCLK1_HZ % fs ) {
		dbprintf("Fractional divide error %i", val );
		m_error = err_fract;
		return m_error;
	}
	dbprintf("Fract divide factor %i", val );
    LL_TIM_InitTypeDef timcfg = {
        .Prescaler = 0,
        .CounterMode = LL_TIM_COUNTERMODE_UP,
        .Autoreload = val,
        .ClockDivision = LL_TIM_CLOCKDIVISION_DIV1,
        .RepetitionCounter =  0
    };
    LL_TIM_Init( AUDIO_TIMER, &timcfg );
    LL_TIM_SetTriggerOutput( AUDIO_TIMER, LL_TIM_TRGO_UPDATE );
    m_state = state::start_wait;
    return m_error;
}

//! Underflow error
void dac_audio::transfer_error( int err )
{
	do_stop();
	if( m_state == state::stop_wait ) {
		m_error = err_success;
	} else {
		m_error = err;
	}
	const auto iret = m_done_sem.signal_isr();
	if( iret != ISIX_EOK ) {
		m_error = iret;
	}
	m_state = state::idle;
}

//! Stop playing
int dac_audio::stop( ostick_t timeout )
{
	if( m_state == state::sampling ) {
		m_state = state::stop_wait;
		m_done_sem.wait( timeout );
		m_error = err_success;
		m_state = state::idle;
		flush_buffers();
		dbprintf("Successfully flushed and stop");
	}
	else if( m_state == state::start_wait ) {
		flush_buffers();
		m_state = state::idle;
		m_error = err_success;
		dbprintf("Flush buffer before play");
	} else {
		dbprintf("No playing");
		m_error = err_not_playing;
	}
	return m_error;
}

//! Raw hardware stop
void dac_audio::do_stop()
{
    LL_DMA_DisableStream( DMA1, DAC_DMA_STREAM );
    LL_TIM_DisableCounter( AUDIO_TIMER );

    LL_DMA_DisableIT_TC( DMA1, DAC_DMA_STREAM );
    LL_DMA_DisableIT_TE( DMA1, DAC_DMA_STREAM );
    LL_DMA_ClearFlag_TC5( DMA1 );
}

//!Play audio with selected sample ratio
int dac_audio::do_play()
{
	if( m_state != state::start_wait ) {
		dbprintf("Invalid state");
		m_error = err_not_playing;
		return m_error;
	}
	void *abuf;
	m_error = m_fifo.pop_isr( abuf );
	if( m_error != ISIX_EOK ) {
		dbprintf("Buffer probably not prepared err %i", m_error );
		return m_error;
	}
	void *abuf1;
	m_error = m_fifo.pop_isr( abuf1 );
	if( m_error != ISIX_EOK ) {
		dbprintf("Buffer probably not prepared err %i", m_error );
		return m_error;
	}
	//Setup semaphore for fin wait
	m_done_sem.trywait();

	// Initialize the DMA
    LL_DMA_InitTypeDef dma_cfg = {
       .PeriphOrM2MSrcAddress =  LL_DAC_DMA_GetRegAddr(DAC1, DAC_CHANNEL, LL_DAC_DMA_REG_DATA_12BITS_LEFT_ALIGNED),
       .MemoryOrM2MDstAddress = reinterpret_cast<uintptr_t>(abuf),
       .Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH,
       .Mode =  LL_DMA_MODE_CIRCULAR,
       .PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT,
       .MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT,
       .PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_HALFWORD,
       .MemoryOrM2MDstDataSize =  LL_DMA_MDATAALIGN_HALFWORD,
       .NbData = m_mempool_len,
       .Channel = DAC_DMA_CHANNEL,
       .Priority = LL_DMA_PRIORITY_MEDIUM,
       .FIFOMode = LL_DMA_FIFOMODE_DISABLE,
       .FIFOThreshold = 0,
       .MemBurst = LL_DMA_MBURST_SINGLE,
       .PeriphBurst = LL_DMA_PBURST_SINGLE
    };
    LL_DMA_Init( DMA1, DAC_DMA_STREAM, &dma_cfg );
    LL_DMA_SetCurrentTargetMem( DMA1, DAC_DMA_STREAM, LL_DMA_CURRENTTARGETMEM1);
    LL_DMA_SetMemory1Address( DMA1, DAC_DMA_STREAM, reinterpret_cast<uintptr_t>(abuf1) );
    LL_DMA_EnableDoubleBufferMode( DMA1, DAC_CHANNEL );

    LL_DMA_EnableIT_TC( DMA1, DAC_DMA_STREAM );
    LL_DMA_EnableIT_TE( DMA1, DAC_DMA_STREAM );
	//Enable DMA and enable timer ready for transfer
    LL_DMA_EnableStream( DMA1, DAC_DMA_STREAM );
    LL_DAC_EnableDMAReq( DAC1, DAC_CHANNEL );
    LL_TIM_EnableCounter( AUDIO_TIMER );
    LL_DAC_Enable( DAC1, DAC_CHANNEL );
	m_error = err_success;
	m_state = state::sampling;
	return m_error;
}

//! Commit filled application buffer
int dac_audio::commit_buffer( uint16_t* buf, ostick_t tout )
{
	m_error = m_fifo.push( buf, tout );
	if( m_state == state::idle || m_state == state::stop_wait ) {
		dbprintf("Buffer curently not playing audio %i", m_state );
		m_error = err_not_playing;
		return m_error;
    }
    if( m_error == ISIX_EOK ) {
		if( m_state == state::start_wait  && m_fifo.size() >= 2 ) {
			dbprintf("Start playing audio");
			do_play();
		}
	} else {
		dbprintf("Fifo push error");
	}
	return m_error;
}

	//! Interrupt stream TC
extern "C" {
void dma1_stream5_isr_vector()
{
	if( g_dac_audio )
	{
        if( LL_DMA_IsActiveFlag_TE5(DMA1) )
		{
			g_dac_audio->free_buffer( reinterpret_cast<void*>(LL_DMA_GetMemoryAddress( DMA1, DAC_DMA_STREAM)) );
			g_dac_audio->free_buffer( reinterpret_cast<void*>(LL_DMA_GetMemory1Address( DMA1, DAC_DMA_STREAM )) );
			g_dac_audio->transfer_error( dac_audio::err_dma );
		}
		else if( LL_DMA_IsActiveFlag_TC5(DMA1) )
		{
			auto ptr = g_dac_audio->get_buffer();
			if( ptr ) {
				if( LL_DMA_GetCurrentTargetMem( DMA1, DAC_DMA_STREAM ) ) {
					//Update DMAR1
                    g_dac_audio->free_buffer( reinterpret_cast<void*>(LL_DMA_GetMemory1Address( DMA1, DAC_DMA_STREAM )) );
					LL_DMA_SetMemory1Address( DMA1, DAC_DMA_STREAM, reinterpret_cast<uintptr_t>(ptr) );
				} else {
					//Update DMAR0
                    g_dac_audio->free_buffer( reinterpret_cast<void*>(LL_DMA_GetMemoryAddress( DMA1, DAC_DMA_STREAM)) );
					LL_DMA_SetMemoryAddress( DMA1, DAC_DMA_STREAM, reinterpret_cast<uintptr_t>(ptr) );
				}
			} else {
                g_dac_audio->free_buffer( reinterpret_cast<void*>(LL_DMA_GetMemoryAddress( DMA1, DAC_DMA_STREAM)) );
                g_dac_audio->free_buffer( reinterpret_cast<void*>(LL_DMA_GetMemory1Address( DMA1, DAC_DMA_STREAM )) );
				g_dac_audio->transfer_error( dac_audio::err_underrun );
			}
		}
	}
    LL_DMA_ClearFlag_TE5( DMA1 );
    LL_DMA_ClearFlag_TC5( DMA1 );
}
}

}
