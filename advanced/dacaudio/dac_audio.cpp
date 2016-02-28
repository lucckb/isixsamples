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

#include <config.h>
#include <dac_audio.hpp>
#include <cstdlib>
#include <new>
#include <stm32gpio.h>
#include <stm32rcc.h>
#include <stm32tim.h>
#include <stm32dac.h>
#include <stm32dma.h>
#include <stm32system.h>
#include <foundation/dbglog.h>
/* ------------------------------------------------------------------ */ 
namespace drv {
/* ------------------------------------------------------------------ */ 
namespace {
	inline void terminate() {
#ifdef __EXCEPTIONS
		throw std::bad_alloc();
#else
		std::abort();
#endif
	}
	//!Configuration DATA
	static constexpr auto DAC_GPIO_PORT = GPIOA;
	static constexpr auto DAC_GPIO_PIN = 4;
	static constexpr auto AUDIO_TIMER = TIM6;
	static constexpr auto AUDIO_TIMER_RCC =  RCC_APB1Periph_TIM6;
	static constexpr auto DAC_DMA_RCC = RCC_AHB1Periph_DMA1;
	static constexpr auto DAC_TRIGER = DAC_Trigger_T6_TRGO;
	static constexpr auto DAC_CHANNEL = DAC_Channel_1;
	static constexpr auto DAC_DMA_STREAM = DMA1_Stream5;
	static constexpr auto DAC_DMA_CHANNEL = DMA_Channel_7;
	static constexpr auto DAC_DMA_STREAM_IRQN = DMA1_Stream5_IRQn;
	// Pointer to the dac audio for interrupt
	dac_audio* g_dac_audio;
}
/* ------------------------------------------------------------------ */
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
	m_errno = hardware_setup();
	if( m_errno ) {
		terminate();
	}
	g_dac_audio = this;
}
/* ------------------------------------------------------------------ */ 
//! Destructor
dac_audio::~dac_audio()
{
	do_stop();
	stm32::nvic_irq_enable( DAC_DMA_STREAM_IRQN, false );
	stm32::rcc_apb1_periph_clock_cmd( RCC_APB1Periph_DAC|AUDIO_TIMER_RCC, false );
	flush_buffers();
	isix::mempool_destroy( m_mempool );
	g_dac_audio = nullptr;
}
/* ------------------------------------------------------------------ */ 
//! Setup the hardware 
int dac_audio::hardware_setup() 
{
	using namespace stm32;
	rcc_ahb1_periph_clock_cmd( RCC_AHB1Periph_GPIOA|DAC_DMA_RCC, true );
	rcc_apb1_periph_clock_cmd( RCC_APB1Periph_DAC|AUDIO_TIMER_RCC, true );
	/* Configure GPIO */
	gpio_config( DAC_GPIO_PORT, DAC_GPIO_PIN, GPIO_MODE_ANALOG, GPIO_PUPD_NONE );
	/* Configure DAC converter */
	dac_init( DAC_CHANNEL, DAC_TRIGER, DAC_WaveGeneration_None, 0, DAC_OutputBuffer_Enable );
	nvic_set_priority( DAC_DMA_STREAM_IRQN, IRQ_PRIO, IRQ_SUB );
	nvic_irq_enable( DAC_DMA_STREAM_IRQN, true );
	return err_success;
}
/* ------------------------------------------------------------------ */
int dac_audio::play( unsigned short fs ) 
{
	if( m_state != state::idle ) {
		dbprintf("Already playing");
		m_errno = err_playing;
		return m_errno;
	}
	//Configure the timer
	static const auto val =  CONFIG_PCLK1_HZ / (fs / 2);
	if( CONFIG_PCLK1_HZ % fs ) {
		dbprintf("Fractional divide error %i", val );
		m_errno = err_fract;
		return m_errno;
	}
	dbprintf("Fract divide factor %i", val );
	stm32::tim_timebase_init( AUDIO_TIMER, 0, TIM_CounterMode_Up, val, 0, 0 );
	stm32::tim_select_output_trigger( AUDIO_TIMER, TIM_TRGOSource_Update );
	m_state = state::start_wait;
	return m_errno;
}
/* ------------------------------------------------------------------ */ 
//! Underflow error
void dac_audio::transfer_error( int err ) 
{
	do_stop();
	if( m_state == state::stop_wait ) {
		m_errno = err_success;
	} else {
		m_errno = err;
	}
	const auto iret = m_done_sem.signal_isr();
	if( iret != ISIX_EOK ) {
		m_errno = iret;
	}
	m_state = state::idle;
}
/* ------------------------------------------------------------------ */ 
//! Stop playing
int dac_audio::stop( ostick_t timeout )  
{
	if( m_state == state::sampling ) {
		m_state = state::stop_wait;
		m_done_sem.wait( timeout );
		m_errno = err_success;
		m_state = state::idle;
		flush_buffers();
		dbprintf("Successfully flushed and stop");
	} 
	else if( m_state == state::start_wait ) {
		flush_buffers();
		m_state = state::idle;
		m_errno = err_success;
		dbprintf("Flush buffer before play");
	} else {
		dbprintf("No playing");
		m_errno = err_not_playing;
	}
	return m_errno;
}
/* ------------------------------------------------------------------ */
//! Raw hardware stop 
void dac_audio::do_stop() 
{
	using namespace stm32;
	dma_cmd( DAC_DMA_STREAM, false );
	tim_cmd( AUDIO_TIMER, false );
	dac_cmd( DAC_CHANNEL, false );
	dma_it_config( DAC_DMA_STREAM, DMA_IT_TC|DMA_IT_TE, false );
	dma_clear_it_pending_bit( DAC_DMA_STREAM , DMA_FLAG_TCIF5 );
}
/* ------------------------------------------------------------------ */ 
//!Play audio with selected sample ratio
int dac_audio::do_play()
{
	using namespace stm32;
	if( m_state != state::start_wait ) {
		dbprintf("Invalid state");
		m_errno = err_not_playing;
		return m_errno;
	}
	void *abuf;
	m_errno = m_fifo.pop_isr( abuf );
	if( m_errno != ISIX_EOK ) {
		dbprintf("Buffer probably not prepared err %i", m_errno );
		return m_errno;
	}
	void *abuf1;
	m_errno = m_fifo.pop_isr( abuf1 );
	if( m_errno != ISIX_EOK ) {
		dbprintf("Buffer probably not prepared err %i", m_errno );
		return m_errno;
	}
	//Setup semaphore for fin wait
	m_done_sem.trywait();
	// Initialize the DMA
	dma_init(DAC_DMA_STREAM, DAC_DMA_CHANNEL | DMA_DIR_MemoryToPeripheral |
			DMA_PeripheralInc_Disable | DMA_MemoryInc_Enable | 
			DMA_PeripheralDataSize_HalfWord | DMA_MemoryDataSize_HalfWord |
			DMA_Mode_Circular | DMA_Priority_High |DMA_MemoryBurst_Single | 
			DMA_PeripheralBurst_Single,
			DMA_FIFOMode_Disable | DMA_FIFOThreshold_HalfFull,
			m_mempool_len, dac_get_channel1_dreg(DAC_Align_12b_L), abuf);
	dma_double_buffer_mode_config( DAC_DMA_STREAM, abuf1, DMA_Memory_1 );
	dma_double_buffer_mode_cmd( DAC_DMA_STREAM, true );
	dma_it_config( DAC_DMA_STREAM, DMA_IT_TC|DMA_IT_TE,  true );
	//Enable DMA and enable timer ready for transfer
	dma_cmd( DAC_DMA_STREAM, true );
	dac_dma_cmd( DAC_CHANNEL, true );
	tim_cmd( AUDIO_TIMER, true );
	dac_cmd( DAC_CHANNEL, true );
	m_errno = err_success;
	m_state = state::sampling;
	return m_errno;
}
/* ------------------------------------------------------------------ */ 
//! Commit filled application buffer
int dac_audio::commit_buffer( uint16_t* buf, ostick_t tout )
{
	m_errno = m_fifo.push( buf, tout );
	if( m_state == state::idle || m_state == state::stop_wait ) {
		dbprintf("Buffer curently not playing audio %i", m_state );
		m_errno = err_not_playing;
		return m_errno;
	}
	if( m_errno == ISIX_EOK ) { 
		if( m_state == state::start_wait  && m_fifo.size() >= 2 ) {
			dbprintf("Start playing audio");
			do_play();
		}
	} else {
		dbprintf("Fifo push error");
	}
	return m_errno;
}
/* ------------------------------------------------------------------ */ 
extern "C" {
	//! Interrupt stream TC
	__attribute__((interrupt)) void dma1_stream5_isr_vector() 
	{
		using namespace stm32;
		if( g_dac_audio ) 
		{
			if( dma_get_flag_status( DAC_DMA_STREAM, DMA_FLAG_TEIF5 ) )
			{
				g_dac_audio->free_buffer( dma_get_memory_target( DAC_DMA_STREAM, DMA_Memory_0 ) );
				g_dac_audio->free_buffer( dma_get_memory_target( DAC_DMA_STREAM, DMA_Memory_1 ) );
				g_dac_audio->transfer_error( dac_audio::err_dma );
			}
			else if( dma_get_flag_status( DAC_DMA_STREAM, DMA_FLAG_TCIF5 ) )
			{
				auto ptr = g_dac_audio->get_buffer();
				if( ptr ) {
					if( dma_get_current_memory_target( DAC_DMA_STREAM ) == 0 ) {
						//Update DMAR1
						g_dac_audio->free_buffer( dma_get_memory_target( DAC_DMA_STREAM, DMA_Memory_1 ) );
						dma_memory_target_config( DAC_DMA_STREAM, ptr, DMA_Memory_1 );
					} else {
						//Update DMAR0
						g_dac_audio->free_buffer( dma_get_memory_target( DAC_DMA_STREAM, DMA_Memory_0 ) );
						dma_memory_target_config( DAC_DMA_STREAM, ptr, DMA_Memory_0 );
					}
				} else {
					g_dac_audio->free_buffer( dma_get_memory_target( DAC_DMA_STREAM, DMA_Memory_0 ) );
					g_dac_audio->free_buffer( dma_get_memory_target( DAC_DMA_STREAM, DMA_Memory_1 ) );
					g_dac_audio->transfer_error( dac_audio::err_underrun );
				}
			}
		}
		dma_clear_flag( DAC_DMA_STREAM, DMA_FLAG_TCIF5|DMA_FLAG_TEIF5 );
	}
}
/* ------------------------------------------------------------------ */ 
}
/* ------------------------------------------------------------------ */
