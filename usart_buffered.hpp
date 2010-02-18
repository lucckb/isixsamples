/*----------------------------------------------------------*/
/*
 * usart_buffered.hpp
 *
 *  Created on: 2009-12-09
 *      Author: lucck
 */
/*----------------------------------------------------------*/
#ifndef USART_BUFFERED_HPP_
#define USART_BUFFERED_HPP_
/*----------------------------------------------------------*/
#include <isix.h>
#include <system.h>
/*----------------------------------------------------------*/
namespace dev
{
/*----------------------------------------------------------*/
extern "C"
{
	void usart1_isr_vector(void) __attribute__ ((interrupt));
	void usart2_isr_vector(void) __attribute__ ((interrupt));
}

/*----------------------------------------------------------*/

class usart_buffered
{
	friend void usart1_isr_vector(void);
	friend void usart2_isr_vector(void);

public:

	enum parity			//Baud enumeration
	{
		parity_none,
		parity_odd,
		parity_even
	};

	//Constructor
	explicit usart_buffered(
		USART_TypeDef *_usart, unsigned cbaudrate = 115200,
		std::size_t queue_size=192, parity cpar=parity_none
	);

	//Set baudrate
	void set_baudrate(unsigned new_baudrate);

	//Set parity
	void set_parity(parity new_parity);

	//Putchar
	int putchar(unsigned char c, int timeout=isix::ISIX_TIME_INFINITE)
	{
		int result = tx_queue.push( c, timeout );
		start_tx();
		return result;
	}

	//Getchar
	int getchar(unsigned char &c, int timeout=isix::ISIX_TIME_INFINITE)
	{
		return rx_queue.pop(c, timeout );
	}

	//Put string into the uart
	int puts(const char *str);

	//Get string into the uart
	int gets(char *str, std::size_t max_len, int timeout=isix::ISIX_TIME_INFINITE);

private:
	static const unsigned IRQ_PRIO = 1;
	static const unsigned IRQ_SUB = 7;

private:
	void start_tx();
	void isr();
	void irq_mask() { ::irq_mask(IRQ_PRIO, IRQ_SUB); }
	void irq_umask() { ::irq_umask(); }

private:
	USART_TypeDef *usart;

	isix::fifo<unsigned char> tx_queue;

	isix::fifo<unsigned char> rx_queue;

	volatile bool tx_en;
};

/*----------------------------------------------------------*/
}
/*----------------------------------------------------------*/

#endif /* USART_BUFFERED_HPP_ */
