/*
 * i2c_master.h
 *
 *  Created on: 19-09-2010
 *      Author: lucck
 */
/* ------------------------------------------------------------------ */
#ifndef I2C_MASTER_H_
#define I2C_MASTER_H_
/* ------------------------------------------------------------------ */
#include <stdint.h>
#include <stdbool.h>
/* ------------------------------------------------------------------ */
enum errno
{
	ERR_OK = 0,						//All is ok
	ERR_BUS = -5000,				//Bus error
	ERR_ARBITRATION_LOST = -5001,
	ERR_ACK_FAILURE = -5002,
	ERR_OVERRUN = - 5003,
	ERR_PEC = - 5004,				//Parity check error
	ERR_BUS_TIMEOUT = -5005, 		//Bus timeout
	ERR_TIMEOUT = - 5006,			//timeout error
	ERR_ISIX = - 5007,				//Isix internal err
	ERR_UNKNOWN = - 5008,
};
typedef enum errno errno_t;
/* ------------------------------------------------------------------ */
//! Initialize the i2c library
errno_t i2cm_init( unsigned clk_speed);
//! Set speed on the bus
void i2cm_set_speed(unsigned speed);
//! Transfer 7 bit on the mag
int i2cm_transfer_7bit(uint8_t addr, const void* wbuffer, short wsize, void* rbuffer, short rsize);
/* ------------------------------------------------------------------ */
#endif /* I2C_MASTER_H_ */
