/*
 * interrupt_cntr.cpp
 *
 *  Created on: 2009-12-09
 *      Author: lucck
 */
/* ------------------------------------------------------------------ */
#include "interrupt_cntr.hpp"

/* ------------------------------------------------------------------ */
namespace dev
{
/* ------------------------------------------------------------------ */
interrupt* interrupt_cntr::vector_tab[irql_last];

const IRQn_Type interrupt_cntr::nvic_irq[] = {
		SysTick_IRQn, USART1_IRQn, USART2_IRQn
};

interrupt_cntr::errno interrupt_cntr::register_int(
		irql irq_no, unsigned  prio, unsigned subprio, interrupt* int_ptr )
{
		if(vector_tab[irq_no]) return e_used;
		vector_tab[irq_no] = int_ptr;
		nvic_set_priority( nvic_irq[irq_no], prio, subprio);
		return e_ok;
}

/* ------------------------------------------------------------------ */
//Call to the global c function
extern "C"
{

void usart1_isr_vector(void) __attribute__ ((interrupt));
void usart1_isr_vector(void)
{
	interrupt_cntr::call_handler(interrupt_cntr::irql_usart1);
}

void usart2_isr_vector(void) __attribute__ ((interrupt));
void usart2_isr_vector(void)
{
	interrupt_cntr::call_handler(interrupt_cntr::irql_usart2);
}

}

/* ------------------------------------------------------------------ */
}

/* ------------------------------------------------------------------ */

