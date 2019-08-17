/*
 * =====================================================================================
 *
 *       Filename:  dac_test.cpp
 *
 *    Description:  DAC audio tester for sine wave
 *
 *        Version:  1.0
 *        Created:  25.11.2014 17:43:17
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Lucjan Bryndza (LB), lucck(at)boff(dot)pl
 *   Organization:  BoFF
 *
 * =====================================================================================
 */
#include <isix.h>
#include <foundation/dbglog.h>
#include <usart_simple.h>
#include <dac_audio.hpp>
#include <cmath>
#include <climits>
#include <cstring>
 
namespace {
//Generate sine wave
void dac_gen( void* ) 
{
	static constexpr auto buf_len = 256UL;
	static constexpr auto srate = 8000UL;
	static constexpr auto freq = 1000UL;
	static constexpr float pi2 = std::atan(1.0)*8.0;
	static constexpr float scale =  32760.0f;
	drv::dac_audio dac { buf_len, 10 };
	auto sinb = new uint16_t[srate/freq];
	//! Generate sinus table
	for( auto i=0UL; i<srate/freq; ++i ) 
	{
		sinb[i] = scale + std::sin(freq*pi2*float(i)/float(srate)) * scale;
		dbprintf("%i=%i",i,sinb[i] );
	}
	dac.play( srate );
	for(;;) 
	{
		auto tbuf = dac.reserve_buffer();
		if( !tbuf ) {
			dbprintf("Fatal nullptr!");
			return;
		}
		for(auto n = 0U; n<buf_len; n+= srate/freq )
			std::memcpy( &tbuf[n], sinb, (srate/freq)*2);
		auto res = dac.commit_buffer( tbuf );
		if( res ) {
			dbprintf("Commit failed %i", res );
			return;
		}
	}
}
  
}
 
//! Main core function tester
int main() {

	dblog_init( stm32::usartsimple_putc, NULL, stm32::usartsimple_init,
	    		USART1, 115200, false, CONFIG_PCLK1_HZ, CONFIG_PCLK2_HZ );

	dbprintf( "PSK core application started" );
	
	//! Crete psk application object
	isix::task_create( dac_gen, nullptr, 2048, isix::get_min_priority(), 0 );

	//! Start isix scheduler
	isix::start_scheduler();
	
	//ISIX scheduler exit
	dbprintf( "PSK application scheduler exit" );

}
