/* ------------------------------------------------------------------ */
/*
 * interrupt_cntr.hpp
 *
 *  Created on: 2009-12-09
 *      Author: lucck
 */

/* ------------------------------------------------------------------ */
#ifndef INTERRUPT_CNTR_HPP_
#define INTERRUPT_CNTR_HPP_
/* ------------------------------------------------------------------ */
#include "system.h"

/* ------------------------------------------------------------------ */
namespace dev
{
/* ------------------------------------------------------------------ */
class interrupt
{
public:
	virtual void isr()=0;
};

/* ------------------------------------------------------------------ */
class interrupt_cntr
{
public:
	enum irql {	irql_timer,		//Timer interrupt
				irql_usart1, 	//Usart0 interrupt
				irql_usart2, 	//Usart1 interrupt
				irql_last
			   };

	enum errno {e_ok, e_used };

	static void call_handler(irql irq_no) { vector_tab[irq_no]->isr(); }

	static errno register_int(irql irq_no, unsigned prio, unsigned subprio, interrupt* int_ptr);

	static void enable_int(irql irq_no)
	{
		nvic_irq_enable( nvic_irq[irq_no],true );
	}

	static void disable_int(irql irq_no)
	{
		nvic_irq_enable( nvic_irq[irq_no],false );
	}

private:
	interrupt_cntr();

	interrupt_cntr(const interrupt_cntr&);

	static interrupt* vector_tab[irql_last];

	static const IRQn_Type nvic_irq[];
};

/* ------------------------------------------------------------------ */
}
#endif /* INTERRUPT_CNTR_HPP_ */
