/*
 * =====================================================================================
 *
 *       Filename:  dac_audio.hpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  27.02.2014 20:03:13
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Lucjan Bryndza (LB), lucck(at)boff(dot)pl
 *   Organization:  BoFF
 *
 * =====================================================================================
 */
#pragma once
#include <foundation/noncopyable.hpp>
#include <isix.h>
/* ------------------------------------------------------------------ */ 
namespace drv {
/* ------------------------------------------------------------------ */
extern "C" {
	__attribute__((interrupt)) void dma1_stream5_isr_vector();
}
/* ------------------------------------------------------------------ */ 
//DAC audio converter class
class dac_audio : public fnd::noncopyable {
	static const auto IRQ_PRIO = 0;			//! IRQ priority
	static const auto IRQ_SUB =  7;			//! IRQ subpriority
	static constexpr auto NR_EXTRA_BUFS = 3; //! Number of extra bufs depends on arch
	friend void dma1_stream5_isr_vector();
public:
	//! Error codes
	enum error {
		err_success = 0, 		//! Error OK
		err_fract = -256, 		//! Fract div
		err_underrun = -257, 	//!Buffer underrun
		err_not_playing = -258, //!Not playing
		err_playing	= -259,		//!Already playing
		err_dma = -260			//!DMA error
	};
	/**
	 *  Constructor
	 *	@param[in] audio_buf_len  Audio chunk size in bytes ( data is always 16 bit alignement )
	 *	@param[in] num_buffers Number of internal audio buffer used for work
	 *	@throw Exception if unable to construct the object
	 */
	dac_audio( size_t audio_buf_len, size_t num_buffers );
	//! Destructor
	~dac_audio();
	/** Start playing audio with selected rate
	 * @param[in] Sample rate 
	 * @return Error code @see error
	 */
	int play( unsigned short fs );
	/** Stop play audio and wait for internal buffers flush
	 * @param[in] timeout Wait timeout
	 * @return Error code @see error
	 */
	int stop( isix::tick_t timeout = isix::ISIX_TIME_INFINITE );
	/** Get the audio buffer for preparation 
	 * @return buffer pointer or null if failed 
	 */
	uint16_t* reserve_buffer() {
		return reinterpret_cast<uint16_t*> 
			( isix::isix_mempool_alloc( m_mempool ) );
	}
	/** Commit filled buffe to the output audio engine 
	 * @param[in] buf Buffer pointer
	 * @param[in] tout Optional timeout
	 * @return Error code @see errora
	 */
	int commit_buffer( uint16_t* buf, isix::tick_t tout = isix::ISIX_TIME_INFINITE );
	//! Get chunk size
	size_t buflen() const {
		return m_mempool_len;
	}
	//! Get errno
	int errno() const {
		return m_errno;
	}
private:
	//Free used bufer
	void free_buffer( void* buf ) {
		isix::isix_mempool_free( m_mempool, buf );
	}
	//Get new buffer
	void* get_buffer() {
		void* ret;
		if( (m_errno=m_fifo.pop_isr( ret ))== isix::ISIX_EOK ) {
			return ret;
		} else {
			return nullptr;
		}
	}
	//! Underflow error
	void transfer_error( int err );
	//! Configure hardware for playing
	int do_play();
	//! Stop internal
	void do_stop();
	//! Setup the hardware
	int hardware_setup();
	//! Flush buffers
	void flush_buffers() {
		void* ret;
		while( m_fifo.pop_isr(ret) == isix::ISIX_EOK ) {
			isix::isix_mempool_free( m_mempool, ret );
		}
	}
private:
	//! DAC converter state
	enum class state : char {
		idle,					//! Engine in idle state
		start_wait,				//! Wait for two buffers
		sampling,				//! Application sampling
		stop_wait				//! Wait for app stop
	};
private:
	isix::isix_mempool_t m_mempool; 		//! Internal mempool for buffer
	isix::fifo<void*> m_fifo;				//! Fin notify fifo
	isix::semaphore m_done_sem;				//! Transfer finish sem
	const unsigned short m_mempool_len;	//! Internal mempool size
	volatile short m_errno {};				//! Last errno
	volatile state m_state { state::idle };	//! Internal application state
};

/* ------------------------------------------------------------------ */ 
}
/* ------------------------------------------------------------------ */ 
