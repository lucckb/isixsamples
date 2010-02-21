/* ------------------------------------------------------------------ */
/*
 * key_transmitter.hpp
 * The serial key transmitter demo
 *  Created on: 2010-02-21
 *      Author: lucck
 */
/* ------------------------------------------------------------------ */
#ifndef KEY_TRANSMITTER_HPP_
#define KEY_TRANSMITTER_HPP_
/* ------------------------------------------------------------------ */
#include <isix.h>
/* ------------------------------------------------------------------ */
//Forward decl.
namespace dev
{
	class usart_buffered;
}

/* ------------------------------------------------------------------ */
namespace app
{

/* ------------------------------------------------------------------ */
//Key tp serial transmitter class
class key_transmitter: public isix::task_base
{
public:
	//Constructor
	key_transmitter(dev::usart_buffered &_serial);
protected:
	//Main thread method
	virtual void main();
private:
	//Stack and prio cfgs
	static const unsigned STACK_SIZE = 256;
	static const unsigned TASK_PRIO = 3;
	//The usart obj ref
	dev::usart_buffered &serial;
};
/* ------------------------------------------------------------------ */
}
/* ------------------------------------------------------------------ */
#endif /* KEY_TRANSMITTER_HPP_ */
/* ------------------------------------------------------------------ */
