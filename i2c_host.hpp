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
/* ------------------------------------------------------------------ */
namespace dev
{
/* ------------------------------------------------------------------ */

class i2c_host;

/* ------------------------------------------------------------------ */
class i2c_interrupt
{
public:
	i2c_interrupt();
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
	i2c_host(I2C_TypeDef * const _i2c);
	int i2c_write_7bit(uint8_t addr, const void *buffer, std::size_t size);
	int i2c_read_7bit(uint8_t addr, void *buffer, std::size_t size);
private:

	enum addr_dir { direction_read, direction_write };
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

	void send_7bit_addr(uint8_t addr, addr_dir direction)
	{
		//Address flag
		static const unsigned OAR1_ADD0_BIT = 1;
		if(direction != direction_write)
			addr |= OAR1_ADD0_BIT;
		else
			addr &= ~OAR1_ADD0_BIT;
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

private:
	I2C_TypeDef *i2c;
};
/* ------------------------------------------------------------------ */
}
/* ------------------------------------------------------------------ */
#endif /* I2C_HOST_HPP_ */

/* ------------------------------------------------------------------ */
