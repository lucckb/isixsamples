/* ------------------------------------------------------------------ */
/*
 * led_receiver.hpp
 *
 *  Created on: 2010-01-02
 *      Author: lucck
 */
/* ------------------------------------------------------------------ */
#ifndef LED_RECEIVER_HPP_
#define LED_RECEIVER_HPP_
/* ------------------------------------------------------------------ */
#include <isix.h>
/* ------------------------------------------------------------------ */
//Fwd class decl
namespace dev
{
	class usart_buffered;
}
/* ------------------------------------------------------------------ */
namespace app
{
//Serial receiver task class
class led_receiver: public isix::task_base
{
public:
	//Constructor
	led_receiver(dev::usart_buffered &_serial);
protected:
	//Main thread method
	virtual void main();
private:
	//Stack configuration
	static const unsigned STACK_SIZE = 256;
	static const unsigned TASK_PRIO = 3;
	//The usart obj ref
	dev::usart_buffered &serial;
};
/* ------------------------------------------------------------------ */
}
/* ------------------------------------------------------------------ */
#endif /* BLINKER_HPP_ */
/* ------------------------------------------------------------------ */
