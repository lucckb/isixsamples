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
namespace app
{

class ledblink: public isix::task_base
{
public:
	ledblink();
	void start() {
		start_thread(STACK_SIZE, TASK_PRIO);
	}
protected:
	virtual void main();
private:
	static const unsigned STACK_SIZE = 256;
	static const unsigned TASK_PRIO = 3;
};
/* ------------------------------------------------------------------ */
}
/* ------------------------------------------------------------------ */
#endif /* BLINKER_HPP_ */
/* ------------------------------------------------------------------ */
