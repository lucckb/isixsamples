/* ------------------------------------------------------------------ */
/*
 * display_server.hpp
 * Display server class header
 *  Created on: 2010-01-03
 *      Author: lucck
 */
/* ------------------------------------------------------------------ */
#ifndef DISPLAY_SERVER_HPP_
#define DISPLAY_SERVER_HPP_
/* ------------------------------------------------------------------ */
#include <isix.h>
#include "nokia_display.hpp"
#include "display_proto.hpp"
/* ------------------------------------------------------------------ */
//The application namespace
namespace app
{

/* ------------------------------------------------------------------ */
//Display server class
class display_server: public isix::task_base
{
public:

	//Constructor
	display_server();

	//Send text message
	void send_message(const display_msg &msg )
	{
		io_fifo.push(&msg);
	}
	//Start server
	void start() {
		start_thread( STACK_SIZE, TASK_PRIO );
	}
protected:
	//Thread/task method
	virtual void main();
private:
	//The display object
	drv::nokia_display display;
	static const unsigned STACK_SIZE = 256;
	static const unsigned TASK_PRIO = 3;
	static const unsigned QUEUE_CAPACITY = 10;
	//The message queue
	isix::fifo<const display_msg*> io_fifo;
};

/* ------------------------------------------------------------------ */

}
/* ------------------------------------------------------------------ */
#endif /* DISPLAY_SERVER_HPP_ */

/* ------------------------------------------------------------------ */
