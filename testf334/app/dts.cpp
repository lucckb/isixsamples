#include <periph/dt/dts.hpp>
#include <periph/gpio/gpio_numbers.hpp>
#include <periph/dt/dts_config.hpp>



namespace periph {
namespace dt {
namespace _dts_config {

namespace {
	constexpr clock clk[] {
		{ bus::ahb1, 72000000 },
		{ bus::apb1, 30000000 },
		{ bus::apb2, 60000000 },
		{}
	};

	constexpr pin ser0_pins[] {
		{ pinfunc::sck, gpio::num::PA12 },
		{ pinfunc::miso, gpio::num::PA13 },
		{ pinfunc::mosi, gpio::num::PA15 },
	{}
	};

	constexpr device devices[]
	{
		{
			"serial0", 0, bus::apb1, 7, 1, ser0_pins
		}, {
			"serial1", 0, bus::apb1, 7, 2, ser0_pins
		},
		{}
	};
}

//! The machine config
constexpr configuration the_machine_config {
	clk,
	devices
};


} } }
