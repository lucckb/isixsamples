/*
 * ledkey.hpp
 *
 *  Created on: 2010-01-02
 *      Author: lucck
 */

#ifndef LEDKEY_HPP_
#define LEDKEY_HPP_

#include <isix.h>

namespace app
{

class ledkey: public isix::task_base
{
public:
	ledkey();
protected:
	virtual void main();

private:
		static const unsigned STACK_SIZE = 256;
		static const unsigned TASK_PRIO = 3;
		bool is_enabled;
};

}

#endif /* LEDKEY_HPP_ */
