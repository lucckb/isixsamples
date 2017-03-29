/*
 * config.hpp
 *
 *  Created on: 2010-01-28
 *      Author: lucck
 */

#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_


#if defined(STM32MCU_MAJOR_TYPE_F1)
//HCLK system speed
#define  CONFIG_XTAL_HZ   8000000
#define  CONFIG_HCLK_HZ   72000000
#define	 CONFIG_PCLK1_HZ (CONFIG_HCLK_HZ/2)
#define  CONFIG_PCLK2_HZ (CONFIG_HCLK_HZ/1)

#elif defined(STM32MCU_MAJOR_TYPE_F4)
//HCLK system speed
#define  CONFIG_XTAL_HZ   8000000
#define  CONFIG_HCLK_HZ   168000000
#define	 CONFIG_PCLK1_HZ (CONFIG_HCLK_HZ/4)
#define  CONFIG_PCLK2_HZ (CONFIG_HCLK_HZ/2)
#elif defined(STM32MCU_MAJOR_TYPE_F2)

//HCLK system speed
#define  CONFIG_XTAL_HZ   8000000
#define  CONFIG_HCLK_HZ  120000000
#define	 CONFIG_PCLK1_HZ (CONFIG_HCLK_HZ/4)
#define  CONFIG_PCLK2_HZ (CONFIG_HCLK_HZ/2)
#endif

//Enable DMA mode
#define CONFIG_ISIXDRV_SPI_ENABLE_DMAIRQ 0
#define CONFIG_ISIX_SHUTDOWN_API 1

#endif /* CONFIG_HPP_ */
