/*
 * =====================================================================================
 *
 *       Filename:  app_env.cpp
 *
 *    Description:  Application environment
 *
 *        Version:  1.0
 *        Created:  27.06.2014 10:55:48
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Lucjan Bryndza (LB), lucck(at)boff(dot)pl
 *   Organization:  BoFF
 *
 * =====================================================================================
 */
#include "app_env.hpp"
#include <memory>
#include <foundation/drv/storage/i2c_eeprom.hpp>
#include <foundation/drv/storage/fs_env.hpp>
#include <isix.h>
#include <foundation/sys/dbglog.h>


namespace {
	constexpr auto I2CA_EEPROM = 0xA0;
	std::unique_ptr<fnd::i2c_eeprom> m_eeprom;
	std::unique_ptr<fnd::filesystem::fs_env> m_fsenv;
	isix::semaphore m_lock { 1, 1 };
}

namespace app {

//! Init env
void initenv( fnd::drv::bus::ibus& bus )
{
	if( !m_eeprom ) {
		m_eeprom.reset( new fnd::i2c_eeprom( bus, I2CA_EEPROM , fnd::i2c_eeprom::type::m24c128 ) );
		m_fsenv.reset( new fnd::filesystem::fs_env( *m_eeprom ) );
	}
}

//! Set env
int setenv( unsigned env_id, const void* buf, size_t buf_len )
{
	isix::sem_lock _lck( m_lock );
	if( m_fsenv ) {
		return m_fsenv->set( env_id, buf, buf_len );
	} else {
		return env::err_not_init;
	}
}

//! Get env
int getenv(  unsigned env_id, void* buf, size_t buf_len )
{
	isix::sem_lock _lck( m_lock );
	if( m_fsenv ) {
		return m_fsenv->get( env_id, buf, buf_len );
	} else {
		return env::err_not_init;
	}
}

//! Unset env
int unsetenv( unsigned env_id )
{
	isix::sem_lock _lck( m_lock );
	if( m_fsenv ) {
		return m_fsenv->unset( env_id );
	} else {
		return env::err_not_init;
	}
}

/** Delete all envirnment variables
 *  and reformat filesystem */
int clearenv( )
{
	isix::sem_lock _lck( m_lock );
	if( m_fsenv ) {
		return m_fsenv->format();
	} else {
		return env::err_not_init;
	}
}

}

