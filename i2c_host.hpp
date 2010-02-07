/* ------------------------------------------------------------------ */
/*
 * i2c_host.hpp
 *
 *  Created on: 2010-01-20
 *      Author: lucck
 */
/* ------------------------------------------------------------------ */
#ifndef I2C_HOST_HPP_
#define I2C_HOST_HPP_
/* ------------------------------------------------------------------ */
#include <system.h>
#include <stm32f10x_lib.h>
#include <stdint.h>
#include <cstddef>
#include <isix.h>
#include "interrupt_cntr.hpp"
/* ------------------------------------------------------------------ */
namespace dev
{
/* ------------------------------------------------------------------ */

class i2c_host;

/* ------------------------------------------------------------------ */
class i2c_interrupt : public interrupt
{
public:
	i2c_interrupt(i2c_host &_owner);
	virtual void isr();
	void irq_mask() { ::irq_mask(IRQ_PRIO, IRQ_SUB); }
	void irq_umask() { ::irq_umask(); }

private:
	i2c_host &owner;
	static const unsigned IRQ_PRIO = 1;
	static const unsigned IRQ_SUB = 7;
};

/* ------------------------------------------------------------------ */
class i2c_host
{
	friend class i2c_interrupt;
public:
	enum errno
	{
		ERR_OK = 0
	};
public:
	i2c_host(I2C_TypeDef * const _i2c, unsigned clk_speed=100000);
	int i2c_write_7bit(uint8_t addr, const void *buffer, unsigned short size, bool stop=true );
	int i2c_read_7bit(uint8_t addr, void *buffer, unsigned short size);
private:


	static const unsigned CR1_ACK_BIT = 0x0400;
	static const unsigned CR1_START_BIT = 0x0100;
	static const unsigned CR1_STOP_BIT = 0x0200;


	uint32_t get_last_event()
	{
		static const uint32_t sflag_mask = 0x00FFFFFF;
		return ( static_cast<uint32_t>(i2c->SR1) |
			     static_cast<uint32_t>(i2c->SR2)<<16 )
			     & sflag_mask;
	}

	void send_7bit_addr(uint8_t addr)
	{

		i2c->DR = addr;
	}

	void send_data(uint8_t data)
	{
		i2c->DR = data;
	}
	uint8_t receive_data()
	{
		return i2c->DR;
	}

	void cr1_reg(unsigned bit, bool en)
	{
		if(en) i2c->CR1 |= bit;
		else i2c->CR1 &= ~bit;
	}

	void ack_on(bool on)
	{
		cr1_reg( CR1_ACK_BIT, on );
	}

	void generate_start(bool en=true)
	{
		cr1_reg( CR1_START_BIT, en );
	}

	void generate_stop(bool en=true)
	{
		cr1_reg( CR1_STOP_BIT, en );
	}

	void set_speed(unsigned speed);

private:
	I2C_TypeDef *i2c;
	union
	{
		const uint8_t *ro_buf;
		uint8_t *rw_buf;
	};
	isix::semaphore sem_lock;
	volatile uint8_t bus_addr;
	volatile bool stop_required;
	volatile unsigned short remaining_bytes;
	volatile unsigned short buf_pos;
	i2c_interrupt interrupt;
};
/* ------------------------------------------------------------------ */
}
/* ------------------------------------------------------------------ */
#endif /* I2C_HOST_HPP_ */

/* ------------------------------------------------------------------ */
