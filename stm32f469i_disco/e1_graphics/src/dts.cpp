#include <periph/dt/dts.hpp>
#include <periph/gpio/gpio_numbers.hpp>
#include <periph/dt/dts_config.hpp>
#include <periph/memory/sdram_dtypes.hpp>
#include <periph/drivers/display/rgb/display_config.hpp>
#include <isix/arch/irq.h>
#include <stm32_ll_usart.h>
#include <stm32_ll_gpio.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_rcc.h>



namespace periph::dt::_dts_config {

namespace {
	constexpr clock clk[] {
		{	bus::ahb1, []() -> unsigned {
			 LL_RCC_ClocksTypeDef clk;
			  LL_RCC_GetSystemClocksFreq(&clk);
			  return clk.HCLK_Frequency;
			}
		},
		{	bus::apb1, []() -> unsigned {
			 LL_RCC_ClocksTypeDef clk;
			  LL_RCC_GetSystemClocksFreq(&clk);
			  return clk.PCLK1_Frequency;
			}
		},
		{	bus::apb2, []() -> unsigned {
			 LL_RCC_ClocksTypeDef clk;
			  LL_RCC_GetSystemClocksFreq(&clk);
			  return clk.PCLK2_Frequency;
			}
		},
		{	bus::cpu, []() -> unsigned {
			 LL_RCC_ClocksTypeDef clk;
			  LL_RCC_GetSystemClocksFreq(&clk);
			  return clk.SYSCLK_Frequency;
			}
		},
		{}
	};

	//Serial debug interface
	constexpr pin ser0_pins[] {
		{ pinfunc::txd, gpio::num::PB10 },
		{}
	};

	// FMC SDRAM pins
	constexpr pin sdram_pins[] {
		{ pinfunc::fmc_sdcke1, gpio::num::PB5 },
		{ pinfunc::fmc_sdne1, gpio::num::PB6 },
		{ pinfunc::fmc_sdnwe, gpio::num::PC0 },
		{ pinfunc::fmc_d2, gpio::num::PD0 },
		{ pinfunc::fmc_d3, gpio::num::PD1 },
		{ pinfunc::fmc_d13, gpio::num::PD8 },
		{ pinfunc::fmc_d14, gpio::num::PD9 },
		{ pinfunc::fmc_d15, gpio::num::PD10 },
		{ pinfunc::fmc_d0, gpio::num::PD14 },
		{ pinfunc::fmc_d1, gpio::num::PD15 },

		{ pinfunc::fmc_nbl0, gpio::num::PE0 },
		{ pinfunc::fmc_nbl1, gpio::num::PE1 },
		{ pinfunc::fmc_d4, gpio::num::PE7 },
		{ pinfunc::fmc_d5, gpio::num::PE8 },
		{ pinfunc::fmc_d6, gpio::num::PE9 },
		{ pinfunc::fmc_d7, gpio::num::PE10 },
		{ pinfunc::fmc_d8, gpio::num::PE11 },
		{ pinfunc::fmc_d9, gpio::num::PE12 },
		{ pinfunc::fmc_d10, gpio::num::PE13 },
		{ pinfunc::fmc_d11, gpio::num::PE14 },
		{ pinfunc::fmc_d12, gpio::num::PE15 },

		{ pinfunc::fmc_a0, gpio::num::PF0 },
		{ pinfunc::fmc_a1, gpio::num::PF1 },
		{ pinfunc::fmc_a2, gpio::num::PF2 },
		{ pinfunc::fmc_a3, gpio::num::PF3 },
		{ pinfunc::fmc_a4, gpio::num::PF4 },
		{ pinfunc::fmc_a5, gpio::num::PF5 },
		{ pinfunc::fmc_sdnras, gpio::num::PF11 },
		{ pinfunc::fmc_a6, gpio::num::PF12 },
		{ pinfunc::fmc_a7, gpio::num::PF13 },
		{ pinfunc::fmc_a8, gpio::num::PF14 },
		{ pinfunc::fmc_a9, gpio::num::PF15 },

		{ pinfunc::fmc_a10, gpio::num::PG0 },
		{ pinfunc::fmc_a11, gpio::num::PG1 },
		{ pinfunc::fmc_a14, gpio::num::PG4 },
		{ pinfunc::fmc_a15, gpio::num::PG5 },
		{ pinfunc::fmc_sdclk, gpio::num::PG8 },
		{ pinfunc::fmc_sdncas, gpio::num::PG15 },

		{ pinfunc::fmc_sdcke0, gpio::num::PH2 },
		{ pinfunc::fmc_sdne0, gpio::num::PH3 },
		{ pinfunc::fmc_d16, gpio::num::PH8 },
		{ pinfunc::fmc_d17, gpio::num::PH9 },
		{ pinfunc::fmc_d18, gpio::num::PH10 },
		{ pinfunc::fmc_d19, gpio::num::PH11 },
		{ pinfunc::fmc_d20, gpio::num::PH12 },
		{ pinfunc::fmc_d21, gpio::num::PH13 },
		{ pinfunc::fmc_d22, gpio::num::PH14 },
		{ pinfunc::fmc_d23, gpio::num::PH15 },

		{ pinfunc::fmc_d24, gpio::num::PI0 },
		{ pinfunc::fmc_d25, gpio::num::PI1 },
		{ pinfunc::fmc_d26, gpio::num::PI2 },
		{ pinfunc::fmc_d27, gpio::num::PI3 },
		{ pinfunc::fmc_nbl2, gpio::num::PI4 },
		{ pinfunc::fmc_nbl3, gpio::num::PI5 },
		{ pinfunc::fmc_d28, gpio::num::PI6 },
		{ pinfunc::fmc_d29, gpio::num::PI7 },
		{ pinfunc::fmc_d30, gpio::num::PI9 },
		{ pinfunc::fmc_d31, gpio::num::PI10 },
	};

	constexpr memory::sdram sdram_conf {
		dt::device_conf_base{},
		/*.sdrtr =*/ 1385,
		/*.nrfs =*/ 7,
		/*.bank1 =*/ {
			.sdcr = { 1, 1, 2, 0, 3, 1, 2, 1, 0 },
			.sdtr = { 2, 2, 2, 7, 4, 7, 2 },
			.sdcmr = { 0 },
		},
		/*.bank2 =*/ {},
	};

	//! LCD pins
	constexpr pin dsi_pins[] {
		{ pinfunc::dsi_te, gpio::num::PJ2 },
		{}
	};

	//! LCD pins
	constexpr pin lcd_pins[] {
		{ pinfunc::lcd_reset, gpio::num::PH7 },
		{ pinfunc::lcd_backlight, gpio::num::PA3 },
		{}
	};
	
	constexpr display::layer_info disp_layer0 {
		.width = 480,
		.height = 800,
		.hsync = 120,
		.vsync = 12,
		.vfp = 12,
		.hfp = 120,
		.vbp = 12,
		.hbp = 120,
		.bpp = 24,
		.base = 0xc000'0000,
	};


	constexpr display::fb_info fb_info {
		{},
		 &disp_layer0, 1
	};

	//SPI controller
	constexpr pin i2c1_pins[] {
		{ pinfunc::scl, gpio::num::PB8 },	//SCK pin
		{ pinfunc::sda, gpio::num::PB9 },	//SCL pin
		{}
	};

	constexpr device_conf i2c1_conf {
		{},
		I2C1_EV_IRQn,
		1,7,					//! IRQ prio subprio
		device_conf::fl_dma		//! Use DMA transfer
		//0
	};


	constexpr device devices[]
	{
		{
			"serial0", USART3_BASE,
			bus::apb1, LL_GPIO_AF_7,
			RCC_APB1ENR_USART3EN_Pos,
			ser0_pins,
			nullptr
		},
		{
			"sdram", 0,
			bus::ahb3, LL_GPIO_AF_12,
			RCC_AHB3ENR_FMCEN_Pos,
			sdram_pins,
			&sdram_conf
		},
		{
			"dsi", DSI_BASE,
			bus::apb2, LL_GPIO_AF_13,
			RCC_APB2ENR_DSIEN_Pos,
			dsi_pins,
			&fb_info
		},
		{
			"ltdc", LTDC_BASE,
			bus::apb2, 0,
			RCC_APB2ENR_LTDCEN_Pos,
			{},
			&fb_info
		},
		{
			"display", 0,
			bus::unspec, 0, 0,
			lcd_pins,
			&fb_info
		},
		{
			"i2c1", reinterpret_cast<uintptr_t>(I2C1),
			bus::apb1, LL_GPIO_AF_4,
			RCC_APB1ENR_I2C1EN_Pos,
			i2c1_pins,
			&i2c1_conf
		},
		{}
	};
}

//! The machine config
constexpr configuration the_machine_config {
	clk,
	devices
};

}
