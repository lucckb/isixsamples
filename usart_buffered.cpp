/*----------------------------------------------------------*/
/*
 * usart_buffered.cpp
 *
 *  Created on: 2009-12-09
 *      Author: lucck
 */
/*----------------------------------------------------------*/
#include "usart_buffered.hpp"
#include <system.h>
#include "config.h"
/*----------------------------------------------------------*/
namespace dev
{

/*----------------------------------------------------------*/
namespace
{
	const unsigned USART1_TX_BIT = 9;
	const unsigned USART1_RX_BIT = 10;
	const unsigned USART2_TX_BIT = 2;
	const unsigned USART2_RX_BIT = 3;
	const unsigned CR1_UE_SET = 0x2000;
	const unsigned CCR_ENABLE_SET = 0x00000001;
	const unsigned USART1_DR_BASE = 0x40013804;
	const unsigned USART_M_BIT = (1<<12);
	const unsigned USART_PCE_BIT = (1<<10);
	const unsigned USART_PS_BIT = (1<<9);
	const unsigned USART_TXEIE = (1<<7);
	const unsigned USART_RXNEIE = (1<<5);
	const unsigned USART_RXNE = (1<<5);
	const unsigned USART_TXE = (1<<7);
	const unsigned USART_ORE = (1<<3);
	const unsigned USART_TC = (1<<6);
	const unsigned USART_FE = (1<<1);
	const unsigned USART_PE = (1<<0);
}

/*----------------------------------------------------------*/
#if 0
//Usart interrupt constructor
usart_interrupt::usart_interrupt(usart_buffered &_owner):owner(_owner)
{
	interrupt_cntr::irql irq_no = interrupt_cntr::irql_last;
	if(owner.usart == USART1)
	{
		irq_no = interrupt_cntr::irql_usart1;
	}
	else if(owner.usart == USART2)
	{
		irq_no = interrupt_cntr::irql_usart2;
	}
	interrupt_cntr::register_int(irq_no,IRQ_PRIO, IRQ_SUB, this );
}
#endif

/*----------------------------------------------------------*/
//! Constructor called for usart buffered
usart_buffered::usart_buffered(USART_TypeDef *_usart, unsigned cbaudrate,
		std::size_t queue_size ,parity cpar
) : usart(_usart), tx_queue(queue_size),
   rx_queue(queue_size) , tx_en( false )
{
	if(_usart == USART1)
	{
		RCC->APB2ENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1;
		//Configure GPIO port TxD and RxD
		io_config(GPIOA,USART1_TX_BIT,GPIO_MODE_10MHZ,GPIO_CNF_ALT_PP);
		io_config(GPIOA,USART1_RX_BIT,GPIO_MODE_INPUT,GPIO_CNF_IN_FLOAT);
	}
	else if(_usart == USART2)
	{
		RCC->APB2ENR |= RCC_APB2Periph_GPIOA;
		RCC->APB1ENR |= RCC_APB1Periph_USART2;
		//Configure GPIO port TxD and RxD
		io_config(GPIOA,USART2_TX_BIT,GPIO_MODE_10MHZ,GPIO_CNF_ALT_PP);
		io_config(GPIOA,USART2_RX_BIT,GPIO_MODE_INPUT,GPIO_CNF_IN_FLOAT);
	}
	//Enable UART
	usart->CR1 = CR1_UE_SET;
	//Setup default baudrate
	set_baudrate( cbaudrate );
	set_parity( cpar );

	//One stop bit
	usart->CR2 = USART_StopBits_1;

	//Enable receiver and transmitter and anable related interrupts
	usart->CR1 |=  USART_Mode_Rx |USART_RXNEIE | USART_Mode_Tx ;

	if( _usart == USART1 )
	{
		//Enable usart IRQ with lower priority
		interrupt_cntr::enable_int( interrupt_cntr::irql_usart1 );
	}
	else if( _usart == USART2 )
	{
	   //Enable usart IRQ with lower priority
	   interrupt_cntr::enable_int( interrupt_cntr::irql_usart2 );
	}
}

/*----------------------------------------------------------*/
void usart_buffered::set_baudrate(unsigned new_baudrate)
{
	while(!(usart->SR & USART_TC));
	//Calculate baud rate
	uint32_t int_part = ((0x19 * PCLK2_HZ) / (0x04 * new_baudrate));
	uint32_t tmp = (int_part / 0x64) << 0x04;
	uint32_t fract_part = int_part - (0x64 * (tmp >> 0x04));
	tmp |= ((((fract_part * 0x10) + 0x32) / 0x64)) & ((u8)0x0F);
	usart->BRR = tmp;
}

/*----------------------------------------------------------*/
void usart_buffered::set_parity(parity new_parity)
{
	while(!(usart->SR & USART_TC));

	//if usart parity has bit check
	if(new_parity == parity_none)
	{
		usart->CR1 &= ~(USART_M_BIT | USART_PCE_BIT);
	}
	else
	{
		usart->CR1 |= USART_M_BIT | USART_PCE_BIT;
	}

	if(new_parity == parity_odd)
	{
		usart->CR1 |= USART_PS_BIT;
	}
	else
	{
		usart->CR1 &= ~USART_PS_BIT;
	}
}

/*----------------------------------------------------------*/
//! Usart start transmision called by usb endpoint
void usart_buffered::start_tx()
{
	irq.irq_mask();
	if(!tx_en)
	{
    	tx_en = true;
		usart->CR1 |= USART_TXEIE;
	}
	irq.irq_umask();
}

/*----------------------------------------------------------*/
//! Usart interrupt handler
void usart_buffered::isr()
{
	uint16_t usart_sr = owner.usart->SR;
	if( usart_sr & USART_RXNE  )
	{
		//Received data interrupt
		unsigned char ch = owner.usart->DR;
		//fifo_put(&hwnd->rx_fifo,ch);
		owner.rx_queue.push_isr(ch);
	}
	if(owner.tx_en && (usart_sr&USART_TXE) )
	{
		unsigned char ch;
		if( owner.tx_queue.pop_isr(ch) == isix::ISIX_EOK )
		{
			owner.usart->DR = ch;
		}
		else
		{
			owner.usart->CR1 &= ~USART_TXEIE;
			owner.tx_en = false;
		}
	}
}


/*----------------------------------------------------------*/
//Put string
int usart_buffered::puts(const char *str)
{
	int r = isix::ISIX_EOK;
	while(*str)
		if( (r=putchar(*str)) == isix::ISIX_EOK ) str++;
		else return r;
	return r;
}

/*----------------------------------------------------------*/
//get string
int usart_buffered::gets(char *str, std::size_t max_len, int timeout)
{

}

/*----------------------------------------------------------*/
extern "C"
{
	void usart1_isr_vector(void) __attribute__ ((interrupt));
	void usart1_isr_vector(void)
	{

	}

	void usart2_isr_vector(void) __attribute__ ((interrupt));
	void usart2_isr_vector(void)
	{

	}

}

/*----------------------------------------------------------*/
}

/*----------------------------------------------------------*/
