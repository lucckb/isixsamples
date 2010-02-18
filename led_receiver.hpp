/* ------------------------------------------------------------------ */
/*
 * blinker.hpp
 *
 *  Created on: 2010-01-02
 *      Author: lucck
 */
/* ------------------------------------------------------------------ */
#ifndef BLINKER_HPP_
#define BLINKER_HPP_
/* ------------------------------------------------------------------ */
#include <isix.h>
/* ------------------------------------------------------------------ */
namespace dev
{
	class usart_buffered;
}
/* ------------------------------------------------------------------ */
namespace app
{

class led_receiver: public isix::task_base
{
public:
	led_receiver(dev::usart_buffered &_serial);
protected:
	virtual void main();
private:
	static const unsigned STACK_SIZE = 256;
	static const unsigned TASK_PRIO = 3;
	dev::usart_buffered &serial;
};
/* ------------------------------------------------------------------ */
}
/* ------------------------------------------------------------------ */
#endif /* BLINKER_HPP_ */
/* ------------------------------------------------------------------ */
