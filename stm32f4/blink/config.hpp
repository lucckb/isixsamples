/* ------------------------------------------------------------------ */
/*
 * config.hpp
 *
 *  Created on: 2010-01-28
 *      Author: lucck
 */
/* ------------------------------------------------------------------ */
#ifndef CONFIG_HPP_
#define CONFIG_HPP_
/* ------------------------------------------------------------------ */
namespace config
{
	namespace
	{
		//HCLK system speed
		const unsigned XTAL_HZ = 8000000ul;
		const unsigned HCLK_HZ = 168000000ul;
		const unsigned PCLK1_HZ = HCLK_HZ/4;
		const unsigned PCLK2_HZ = HCLK_HZ/2;
	}
}


/* ------------------------------------------------------------------ */
#endif /* CONFIG_HPP_ */
