/*
 * =====================================================================================
 *
 *       Filename:  ethernetif_f4.c
 *
 *    Description:  Ethernet interface for stm32f4
 *
 *        Version:  1.0
 *        Created:  30.08.2016 19:46:43
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Lucjan Bryndza (LB), lbryndza.p(at)boff(dot)pl
 *   Organization:  BoFF
 *
 * =====================================================================================
 */

#include "ethernetif_prv.h"
#include <stm32_ll_rcc.h>
#include <stm32_ll_system.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_gpio.h>

#if ETH_DRV_USE_RMII == 0
#error  MII inteface not implemented for selected platform
#endif

#define bv(x) (1<<(x))


/**
  * @brief  Deinitializes the ETHERNET peripheral registers to their default reset values.
  * @param  None
  * @retval None
  */

void _ethernetif_deinit_arch_(void)
{
	LL_AHB1_GRP1_ForceReset(LL_AHB1_GRP1_PERIPH_ETHMAC);
	asm volatile("dmb\t\n");
	LL_AHB1_GRP1_ReleaseReset(LL_AHB1_GRP1_PERIPH_ETHMAC);
	asm volatile("dmb\t\n");
}


//! GPIO initialize MII setup
void _ethernetif_gpio_mii_init_arch_(void)
{
}


/** RMII ETH GPIO Configuration
    PA1     ------> ETH_REF_CLK
    PA2     ------> ETH_MDIO
    PA7     ------> ETH_CRS_DV
    PB11     ------> ETH_TX_EN
    PB12     ------> ETH_TXD0
    PB13     ------> ETH_TXD1
    PC1     ------> ETH_MDC
    PC4     ------> ETH_RXD0
    PC5     ------> ETH_RXD1
    */
void _ethernetif_gpio_rmii_init_arch_(void)
{
	LL_AHB1_GRP1_EnableClock( LL_AHB1_GRP1_PERIPH_GPIOA|
		LL_AHB1_GRP1_PERIPH_GPIOB | LL_AHB1_GRP1_PERIPH_GPIOC );

	LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_SYSCFG);
	//Configure alternate functions
	//Normal GPIOS
	LL_GPIO_InitTypeDef port_cnf =
	{
		.Pin = LL_GPIO_PIN_1|LL_GPIO_PIN_2|LL_GPIO_PIN_7,
		.Mode = LL_GPIO_MODE_ALTERNATE,
		.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH,
		.OutputType = LL_GPIO_OUTPUT_PUSHPULL,
		.Pull = LL_GPIO_PULL_NO,
		.Alternate = LL_GPIO_AF_11
	};
	LL_GPIO_Init(GPIOA, &port_cnf);
	// Port B
	port_cnf.Pin = LL_GPIO_PIN_11|LL_GPIO_PIN_12|LL_GPIO_PIN_13;
	LL_GPIO_Init(GPIOB, &port_cnf);

	// PortC
	port_cnf.Pin = LL_GPIO_PIN_1|LL_GPIO_PIN_4|LL_GPIO_PIN_5;
	LL_GPIO_Init(GPIOC, &port_cnf);

	//Configure PHY interrupt
	port_cnf.Pin = 1U << PHY_INT_EXTI_NUM;
	port_cnf.Mode = LL_GPIO_MODE_INPUT;
	LL_GPIO_Init(PHY_INT_EXTI_XGPIO(PHY_INT_GPIO_PORT), &port_cnf);
	
	LL_SYSCFG_SetPHYInterface( LL_SYSCFG_PMC_ETHRMII );
}


void _ethernetif_clock_setup_arch_(bool /*enable*/)
{
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_ETHMAC|
		LL_AHB1_GRP1_PERIPH_ETHMACTX | LL_AHB1_GRP1_PERIPH_ETHMACRX);
}



void _ethernetif_dma_setup_arch_(void)
{

}
