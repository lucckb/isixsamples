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
class rtc_reader : public isix::task_base
{
public:
	rtc_reader();
protected:
	//Thread/task method
	virtual void main();
private:
	static const unsigned STACK_SIZE = 256;
	static const unsigned TASK_PRIO = 3;
	dev::i2c_host i2c_bus;
};
/* ------------------------------------------------------------------ */
}
/* ------------------------------------------------------------------ */
#endif /* RTC_READER_HPP_ */
/* ------------------------------------------------------------------ */
