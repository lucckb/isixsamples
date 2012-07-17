/* ------------------------------------------------------------------ */
/*
 * i2c_host.hpp
 *	This is the class provides i2c support into the isix environment
 *  Created on: 2010-01-20
 *      Author: lucck
 */
/* ------------------------------------------------------------------ */
#ifndef I2C_HOST_HPP_
#define I2C_HOST_HPP_
/* ------------------------------------------------------------------ */
#include <system.h>
#include <stm32lib.h>
#include <stdint.h>
#include <cstddef>
#include <isix.h>

/* ------------------------------------------------------------------ */
namespace dev
{
/* ------------------------------------------------------------------ */

//ISR funcs decls
extern "C" {
void i2c1_ev_isr_vector(void) __attribute__ ((interrupt));
void i2c1_er_isr_vector(void) __attribute__ ((interrupt));
}

/* ------------------------------------------------------------------ */
//I2c host class
class i2c_host
{
	//Friend interrupt class
	friend void i2c1_ev_isr_vector(void);
	friend void i2c1_er_isr_vector(void);
public:
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
		ERR_UNKNOWN = - 5007
	};
	//Default constructor
	i2c_host(I2C_TypeDef * const _i2c, unsigned clk_speed=100000);
	//I2c transfer main function
	int i2c_transfer_7bit(uint8_t addr, const void* wbuffer, short wsize, void* rbuffer, short rsize);
private:
	//Interrupt service routine
	void isr();
	void isr_er();
	//Configuration data
	static const unsigned TRANSFER_TIMEOUT = 1000;
	static const unsigned IRQ_PRIO = 1;
	static const unsigned IRQ_SUB = 7;
	//Rest of the data
	static const unsigned CR1_ACK_BIT = 0x0400;
	static const unsigned CR1_START_BIT = 0x0100;
	static const unsigned CR1_STOP_BIT = 0x0200;
	static const uint16_t I2C_IT_BUF = 0x0400;
	static const uint16_t I2C_IT_EVT = 0x0200;
	static const uint16_t I2C_IT_ERR = 0x0100;
	static const uint16_t CR1_PE_SET = 0x0001;
	//Get last i2c event
	uint32_t get_last_event()
	{
		static const uint32_t sflag_mask = 0x00FFFFFF;
		return ( static_cast<uint32_t>(i2c->SR1) |
			     static_cast<uint32_t>(i2c->SR2)<<16 )
			     & sflag_mask;
	}
	//Send 7 bit address on the i2c bus
	void send_7bit_addr(uint8_t addr)
	{
		i2c->DR = addr;
	}
	//Send data on the bus
	void send_data(uint8_t data)
	{
		i2c->DR = data;
	}
	//Read data from the bus
	uint8_t receive_data()
	{
		return i2c->DR;
	}
	//CR1 reg enable disable
	void cr1_reg(unsigned bit, bool en)
	{
		if(en) i2c->CR1 |= bit;
		else i2c->CR1 &= ~bit;
	}
	//ACK ON control
	void ack_on(bool on)
	{
		cr1_reg( CR1_ACK_BIT, on );
	}
	//Generate start
	void generate_start(bool en=true)
	{
		cr1_reg( CR1_START_BIT, en );
	}
	//Generate stop
	void generate_stop(bool en=true)
	{
		cr1_reg( CR1_STOP_BIT, en );
	}
	//Clear data flags (dummy read)
	void clear_flags()
	{
		static_cast<void>(static_cast<volatile uint16_t>(i2c->SR1));
		static_cast<void>(static_cast<volatile uint16_t>(i2c->DR));
	}
	//Control enabling disabling int in the device
	void devirq_on(bool en=true)
	{
		if(en)
			/* Enable I2C interrupt */
			i2c->CR2 |=  I2C_IT_EVT| I2C_IT_ERR;
		else
			/* diasable I2C interrupt */
			 i2c->CR2 &=  ~(I2C_IT_EVT | I2C_IT_ERR);
	}
	//Set bus speed
	void set_speed(unsigned speed);
	//Translate error to the error code
	int get_hwerror();


private:	//Data
	//I2c device number
	I2C_TypeDef *i2c;
	//Tx buffer pointer
	const uint8_t *tx_buf;
	//Rx buffer pointer
	uint8_t *rx_buf;
	//Busy semaphore
	isix::semaphore sem_lock;
	//Read semaphore
	isix::semaphore sem_irq;
	//Bus address
	volatile uint8_t bus_addr;
	//Bus error flags
	volatile uint8_t err_flag;
	//Tx counter
	volatile short tx_bytes;
	//Rx counter
	volatile short rx_bytes;
	//Position in the buffer
	volatile short buf_pos;

private:	//Noncopyable
	i2c_host(i2c_host &);
	i2c_host& operator=(const i2c_host&);
};
/* ------------------------------------------------------------------ */
}
/* ------------------------------------------------------------------ */
#endif /* I2C_HOST_HPP_ */

/* ------------------------------------------------------------------ */
