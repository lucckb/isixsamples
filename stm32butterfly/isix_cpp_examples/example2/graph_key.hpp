/* ------------------------------------------------------------------ */
/*
 * keyboard.hpp
 * Keyboyard send event thread class header
 *  Created on: 2010-01-09
 *      Author: lucck
 */
/* ------------------------------------------------------------------ */
#ifndef KEYBOARD_HPP_
#define KEYBOARD_HPP_
/* ------------------------------------------------------------------ */
#include <isix.h>
#include "display_proto.hpp"
/* ------------------------------------------------------------------ */

//The application namespace
namespace app
{
/* ------------------------------------------------------------------ */
//Forward declaration
class display_server;

/* ------------------------------------------------------------------ */
//Graph key class
class graph_key: public isix::task_base
{
public:
	//Constructor
	graph_key(display_server &_srv);
	void start() {
		start_thread( STACK_SIZE, TASK_PRIO );
	}
protected:
	//The task/thread
	virtual void main() noexcept;

private:
	static const unsigned STACK_SIZE = 256;
	static const unsigned TASK_PRIO = 2;
	//The display server ref.
	display_server &disp_srv;
};
/* ------------------------------------------------------------------ */
}
/* ------------------------------------------------------------------ */
#endif /* KEYBOARD_HPP_ */
/* ------------------------------------------------------------------ */
