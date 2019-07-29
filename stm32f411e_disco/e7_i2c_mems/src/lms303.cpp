#include "lms303.hpp"
#include <periph/drivers/i2c/i2c_master.hpp>
#include <foundation/sys/dbglog.h>

namespace  app {

namespace {
    static constexpr auto LSM303_ADDRESS_ACCEL = 0x32U;
    static constexpr auto LSM303_ADDRESS_MAG = 0x3cU;
    static constexpr auto LMS303_ID = 0b11010100U;
    enum accel {
        LSM303_REGISTER_ACCEL_CTRL_REG1_A         = 0x20,   // 00000111   rw
        LSM303_REGISTER_ACCEL_CTRL_REG2_A         = 0x21,   // 00000000   rw
        LSM303_REGISTER_ACCEL_CTRL_REG3_A         = 0x22,   // 00000000   rw
        LSM303_REGISTER_ACCEL_CTRL_REG4_A         = 0x23,   // 00000000   rw
        LSM303_REGISTER_ACCEL_CTRL_REG5_A         = 0x24,   // 00000000   rw
        LSM303_REGISTER_ACCEL_CTRL_REG6_A         = 0x25,   // 00000000   rw
        LSM303_REGISTER_ACCEL_REFERENCE_A         = 0x26,   // 00000000   r
        LSM303_REGISTER_ACCEL_STATUS_REG_A        = 0x27,   // 00000000   r
        LSM303_REGISTER_ACCEL_OUT_X_L_A           = 0x28,
        LSM303_REGISTER_ACCEL_OUT_X_H_A           = 0x29,
        LSM303_REGISTER_ACCEL_OUT_Y_L_A           = 0x2A,
        LSM303_REGISTER_ACCEL_OUT_Y_H_A           = 0x2B,
        LSM303_REGISTER_ACCEL_OUT_Z_L_A           = 0x2C,
        LSM303_REGISTER_ACCEL_OUT_Z_H_A           = 0x2D,
        LSM303_REGISTER_ACCEL_FIFO_CTRL_REG_A     = 0x2E,
        LSM303_REGISTER_ACCEL_FIFO_SRC_REG_A      = 0x2F,
        LSM303_REGISTER_ACCEL_INT1_CFG_A          = 0x30,
        LSM303_REGISTER_ACCEL_INT1_SOURCE_A       = 0x31,
        LSM303_REGISTER_ACCEL_INT1_THS_A          = 0x32,
        LSM303_REGISTER_ACCEL_INT1_DURATION_A     = 0x33,
        LSM303_REGISTER_ACCEL_INT2_CFG_A          = 0x34,
        LSM303_REGISTER_ACCEL_INT2_SOURCE_A       = 0x35,
        LSM303_REGISTER_ACCEL_INT2_THS_A          = 0x36,
        LSM303_REGISTER_ACCEL_INT2_DURATION_A     = 0x37,
        LSM303_REGISTER_ACCEL_CLICK_CFG_A         = 0x38,
        LSM303_REGISTER_ACCEL_CLICK_SRC_A         = 0x39,
        LSM303_REGISTER_ACCEL_CLICK_THS_A         = 0x3A,
        LSM303_REGISTER_ACCEL_TIME_LIMIT_A        = 0x3B,
        LSM303_REGISTER_ACCEL_TIME_LATENCY_A      = 0x3C,
        LSM303_REGISTER_ACCEL_TIME_WINDOW_A       = 0x3D
    };
    enum mag {
        LSM303_REGISTER_MAG_CRA_REG_M             = 0x00,
        LSM303_REGISTER_MAG_CRB_REG_M             = 0x01,
        LSM303_REGISTER_MAG_MR_REG_M              = 0x02,
        LSM303_REGISTER_MAG_OUT_X_H_M             = 0x03,
        LSM303_REGISTER_MAG_OUT_X_L_M             = 0x04,
        LSM303_REGISTER_MAG_OUT_Z_H_M             = 0x05,
        LSM303_REGISTER_MAG_OUT_Z_L_M             = 0x06,
        LSM303_REGISTER_MAG_OUT_Y_H_M             = 0x07,
        LSM303_REGISTER_MAG_OUT_Y_L_M             = 0x08,
        LSM303_REGISTER_MAG_SR_REG_Mg             = 0x09,
        LSM303_REGISTER_MAG_IRA_REG_M             = 0x0A,
        LSM303_REGISTER_MAG_IRB_REG_M             = 0x0B,
        LSM303_REGISTER_MAG_IRC_REG_M             = 0x0C,
        LSM303_REGISTER_MAG_TEMP_OUT_H_M          = 0x31,
        LSM303_REGISTER_MAG_TEMP_OUT_L_M          = 0x32
    };
}

//! Constructor
lms303::lms303(periph::drivers::i2c_master& i2c)
    : m_i2c(i2c)
{
}
// Configure and initialize system
int lms303::configure()
{
    int ret {};
    do {
        // Enable the accelerometer
        ret = write_reg(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x27);
        if(ret<0) break;
        // Enable the magnetometer
        ret = write_reg(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_MR_REG_M, 0x00);
        if(ret<0) break;
    } while(0);
    return ret;
}

// Get current acceleration
int lms303::get_accel(int& x, int& y, int& z)
{
	constexpr uint8_t raddr = LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80;
	unsigned char readout[6] {};
	int ret {};
    periph::blk::trx_transfer tran(&raddr,readout, sizeof raddr, sizeof readout);
	do {
		ret = m_i2c.transaction(LSM303_ADDRESS_ACCEL,tran);
		if(ret<0) {
			dbg_err("Transaction fail %i",ret);
			break;
		}
	} while(0);
	x = (int16_t)((readout[1] << 8) | readout[0]);
	y = (int16_t)((readout[3] << 8) | readout[2]);
	z = (int16_t)((readout[5] << 8) | readout[4]);
	return ret;
}

//Get current orientation
int lms303::get_orientation(int& x, int& y, int& z)
{
	constexpr uint8_t raddr = LSM303_REGISTER_MAG_OUT_X_H_M;
	int ret {};
	char readout[6] {};
    periph::blk::trx_transfer tran(&raddr,readout, sizeof raddr, sizeof readout);
	do {
		ret = m_i2c.transaction(LSM303_ADDRESS_MAG,tran);
		if(ret<0) {
			dbg_err("Transaction fail %i",ret);
			break;
		}
	} while(0);
	x = (readout[0]<<8)|readout[1];
	y = (readout[2]<<8)|readout[3];
	z = (readout[4]<<8)|readout[5];
	return ret;
}

//Write i2c registers
int lms303::write_reg(int addr, int reg, unsigned char value)
{
    const unsigned char buf[] = { static_cast<unsigned char>(reg), value };
    periph::blk::tx_transfer tran(buf, sizeof buf);
    return m_i2c.transaction(addr, tran);
}

//Read i2c registers
int lms303::read_reg(int addr, int reg, unsigned char& value)
{
    unsigned char baddr = reg&0xff;
    periph::blk::trx_transfer tran(&baddr,&value, sizeof baddr, sizeof value);
    return m_i2c.transaction(addr, tran);
}

}
