/* ------------------------------------------------------------------ */
/*
 * rtc_reader.hpp
 *
 *  Created on: 2010-02-07
 *      Author: lucck
 */
/* ------------------------------------------------------------------ */
#ifndef RTC_READER_HPP_
#define RTC_READER_HPP_
/* ------------------------------------------------------------------ */
#include <isix.h>
#include "i2c_host.hpp"
/* ------------------------------------------------------------------ */
namespace app
{
/* ------------------------------------------------------------------ */
class display_server;
/* ------------------------------------------------------------------ */
class rtc_reader : public isix::task_base
{
public:
	rtc_reader(display_server &disp_srv_);
	void start() {
		start_thread( STACK_SIZE, TASK_PRIO );
	}
protected:
	//Thread/task method
	virtual void main();
private:
	static const unsigned STACK_SIZE = 512;
	static const unsigned TASK_PRIO = 3;
	dev::i2c_host i2c_bus;
	display_server &disp_srv;
};
/* ------------------------------------------------------------------ */
}
/* ------------------------------------------------------------------ */
#endif /* RTC_READER_HPP_ */
/* ------------------------------------------------------------------ */
