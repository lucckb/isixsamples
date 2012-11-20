/* ------------------------------------------------------------------ */
/*
 * config.h
 *
 *  Created on: 18-09-2010
 *      Author: lucck
 */
/* ------------------------------------------------------------------ */
#ifndef CONFIG_H_
#define CONFIG_H_
/* ------------------------------------------------------------------ */
/* HCLK HZ frequency */
#define HCLK_HZ 72000000
/* PCLK1 frequency */
#define PCLK1_HZ (HCLK_HZ/2)
/* PCLK2 frequency */
#define PCLK2_HZ (HCLK_HZ)
/* ------------------------------------------------------------------ */
//Ethernet configuration
#define ETH_DRV_CONFIGURE_MCO_OUTPUT 1
//EXTI line IRQ number
#define PHY_INT_EXTI_LINE_IRQ_N EXTI0_IRQn
//PHI interrupt pin number
#define PHY_INT_EXTI_NUM 0
//Exti port
#define PHY_INT_GPIO_PORT C
/* ------------------------------------------------------------------ */

#endif /* CONFIG_H_ */
