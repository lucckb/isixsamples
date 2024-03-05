/** Example shows howto use fifo from irq context
 *  Hardware platform: STM32F411E-DISCO
 *  PA2 Port - USART TXD should be connected to serial<->usb converter
 */
#include <config/conf.h>
#include <periph/gpio/gpio.hpp>
#include <foundation/sys/dbglog.h>
#include <periph/drivers/serial/uart_early.hpp>
#include <isix.h>
#include <isix/arch/irq_platform.h>
#include <isix/arch/irq.h>
#include <stm32_ll_exti.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_system.h>






extern "C" {
	//! Exti 0 vector
	void exti0_isr_vector() {
	}
}

// Start main function
auto main() -> int
{

}


