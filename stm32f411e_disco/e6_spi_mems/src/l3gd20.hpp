/*
 * =====================================================================================
 *
 *       Filename:  l3gd20.hpp
 *
 *    Description:  L3GD20 simple driver
 *
 *        Version:  1.0
 *        Created:  15.04.2019 15:33:22
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Lucjan Bryndza (LB), lbryndza.p@boff.pl
 *   Organization:  BoFF
 *
 * =====================================================================================
 */

#include <periph/core/block_device.hpp>
#include <cstdint>

namespace app {

	class l3gd20
	{
	public:
		struct value_t {
			short x{}; short y{}; short z{};
		};
		using temp_t = unsigned char;
	public:
		l3gd20(periph::block_device& dev, int cs)
			: m_dev(dev), m_cs(cs)
		{}
		~l3gd20() {}
		l3gd20(l3gd20&) = delete;
		l3gd20& operator=(l3gd20&) = delete;
		//! Enable to default state
		int enable(bool en);
		//! Read gyro
		int read(value_t& val) const;
		//! Read temperature
		int read_temp(temp_t& temp);
	private:
		//! Write data into register
		int write_reg(unsigned addr, uint8_t value);
		//! Read data from the register
		int read_reg(unsigned addr, uint8_t& value);
		//! Check and initialize if it is connected
		int probe();
	private:
		/* data */
		periph::block_device& m_dev;
		const int m_cs;
	};

	//! Compare operator
	static inline bool operator==(const l3gd20::value_t& v1,const l3gd20::value_t& v2) {
		return v1.x==v2.x && v1.y==v2.y && v1.z==v2.z;
	}
	//! Not exact operator
	static inline bool operator!=(const l3gd20::value_t& v1,const l3gd20::value_t& v2) {
		return !(v1==v2);
	}

}
