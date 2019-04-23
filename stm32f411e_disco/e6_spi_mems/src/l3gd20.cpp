/*
 * =====================================================================================
 *
 *       Filename:  l3gd20.cpp
 *
 *    Description:  L3GD20 accell driver
 *
 *        Version:  1.0
 *        Created:  15.04.2019 15:39:04
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Lucjan Bryndza (LB), lbryndza.p@boff.pl
 *   Organization:  BoFF
 *
 * =====================================================================================
 */
#include "l3gd20.hpp"
#include <foundation/sys/dbglog.h>

namespace app {

namespace {
	static constexpr auto  DIR_READ	 = 1<<7;
	static constexpr auto  DIR_WRITE = 0<<7;
	static constexpr auto  ADDR_INCREMENT = 1<<6;
	static constexpr auto ADDR_WHO_AM_I			= 0x0F;
	static constexpr auto WHO_I_AM_H			= 0xD7;
	static constexpr auto WHO_I_AM				= 0xD4;
	static constexpr auto WHO_I_AM_L3G4200D	= 0xD3;	/* for L3G4200D */
	static constexpr auto  L3GD20_CTRL_REG1  =  0x20;
	static constexpr auto  L3GD20_CTRL_REG2  =  0x21;
	static constexpr auto  L3GD20_CTRL_REG3  =  0x22;
	static constexpr auto  L3GD20_CTRL_REG4  =  0x23;
	static constexpr auto  L3GD20_CTRL_REG5  =  0x24;
	static constexpr auto  L3GD20_OUT_X_L = 0x28;
	static constexpr auto L3GD20_OUT_TEMP  = 0x26;

}

//! Write into the register
int l3gd20::write_reg(unsigned addr, uint8_t value)
{
	const uint8_t cmd[2] {uint8_t(addr|DIR_WRITE),value};
	periph::blk::tx_transfer tran(cmd,sizeof cmd);
	return m_dev.transaction(m_cs, tran);
}

//! Read data from the register
int l3gd20::read_reg(unsigned addr, uint8_t& value)
{
	const uint8_t inbuf[2] {uint8_t(addr|DIR_READ),0};
	uint8_t outbuf[2] {};
	static_assert(sizeof outbuf==sizeof inbuf);
	periph::blk::trx_transfer tran(inbuf,outbuf,sizeof outbuf);
	int ret =  m_dev.transaction(m_cs, tran);
	if(ret<0) return ret;
	value = outbuf[1];
	return ret;
}


//! Check and initialize if it is connected
int l3gd20::probe()
{
	uint8_t val;
	int ret;
	do {
		ret = read_reg(ADDR_WHO_AM_I,val);
		if(ret<0) break;
		ret = read_reg(ADDR_WHO_AM_I,val);
		if(ret<0) break;
		if(val==WHO_I_AM) break;
		if(val==WHO_I_AM_H) break;
		if(val==WHO_I_AM_L3G4200D) break;
	} while(0);
	return ret;
}


//! Enable to default state
int l3gd20::enable(bool en)
{
	int ret {};
	do {
		if( en ) {
			if((ret=probe())<0) break;
			if((ret=write_reg(L3GD20_CTRL_REG1,0x0f))<0) break;
			if((ret=write_reg(L3GD20_CTRL_REG2,0x00))<0) break;
			if((ret=write_reg(L3GD20_CTRL_REG3,0b00001000))<0) break;
			if((ret=write_reg(L3GD20_CTRL_REG4,0b00110000))<0) break;
			if((ret=write_reg(L3GD20_CTRL_REG5,0x00))<0) break;
		} else {
			if((ret=write_reg(L3GD20_CTRL_REG1,0x00))<0) break;
		}
	} while(0);
	return ret;
}

//! Read gyro
int l3gd20::read(value_t& val) const
{
	constexpr uint8_t inbuf[7] = { L3GD20_OUT_X_L | DIR_READ |ADDR_INCREMENT };
	uint8_t outbuf[7] {};
	enum {na,xla,xha,yla,yha,zla,zha};
	static_assert( sizeof outbuf == sizeof inbuf );
	periph::blk::trx_transfer tran(inbuf,outbuf,sizeof outbuf);
	int ret = m_dev.transaction(m_cs, tran);
	if(ret<0) return ret;
	val.x = (int16_t(outbuf[xha])<<8)|int16_t(outbuf[xla]);
	val.y = (int16_t(outbuf[yha])<<8)|int16_t(outbuf[yla]);
	val.z = (int16_t(outbuf[zha])<<8)|int16_t(outbuf[zla]);
	return ret;
}

//! Read temperature
int l3gd20::read_temp(temp_t& temp)
{
	return read_reg(L3GD20_OUT_TEMP,temp);
}

}

