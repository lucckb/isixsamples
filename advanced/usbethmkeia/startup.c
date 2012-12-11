/*
 * example1.c
 *
 *  Created on: 18-09-2010
 *      Author: lucck
 */
/* ------------------------------------------------------------------ */
#include <isix.h>
#include <stm32lib.h>
#include <stm32system.h>
#include <stm32rcc.h>
#include "config.h"
/* ------------------------------------------------------------------ */
//Number of isix threads
#define ISIX_NUM_PRIORITIES 4
//SysTimer values
#define MHZ 1000000
#define CTRL_TICKINT_Set 2
#define SysTick_Counter_Enable 1
/* ------------------------------------------------------------------ */

/** Cortex stm32 System setup
 * Clock and flash configuration for selected rate
 */
void uc_periph_setup()
{
	//Enable HSE fait for setup
	rcc_hse_config( RCC_HSE_ON );
	while( !rcc_get_flag_status(RCC_FLAG_HSERDY ) ) nop();
    //Configure flash: Prefetch enable and 1 wait state
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;
    /* HCLK = SYSCLK */
    rcc_hclk_config( RCC_SYSCLK_Div1 );
    /* PCLK2 = HCLK */
    rcc_pclk2_config( RCC_HCLK_Div1 );
    /* PCLK1 = HCLK/2 */
    rcc_pclk1_config( RCC_HCLK_Div2 );
    /* PREDIV2: PREDIV2CLK = HSE / 2 = 5 MHz */
    rcc_prediv2_config( RCC_PREDIV2_Div2 );
    /* PLL3: PLL3CLK = (HSE / 2) * 10 = 50 MHz */
    rcc_pll3_config(RCC_PLL3Mul_10);
    /* Wlacz PLL3 */
    rcc_pll3_cmd(ENABLE);
    while(!rcc_get_flag_status(RCC_FLAG_PLL3RDY)) nop();
    /* MCO: MCO_OUT = PLL3 / 2 = 25 MHz*/
    rcc_mco_config(RCC_MCO_PLL3CLK_Div2);
    /* PLL2: PLL2CLK = (HSE / 2) * 8 = 40 MHz */
    rcc_pll2_config(RCC_PLL2Mul_8);
    rcc_pll2_cmd(ENABLE);
    while(!rcc_get_flag_status(RCC_FLAG_PLL2RDY)) nop();
    /* PREDIV1: PREDIV1CLK = PLL2 / 5 = 8 MHz */
    rcc_prediv1_config(RCC_PREDIV1_Source_PLL2,RCC_PREDIV1_Div5);
    /* PLL: PLLCLK = PREDIV1 * 9 = 72 MHz */
    rcc_pll_config(RCC_PLLSource_PREDIV1, RCC_PLLMul_9);
    /* Wlacz PLL */
    rcc_pll_cmd(ENABLE);
    while(!rcc_get_flag_status(RCC_FLAG_PLLRDY)) nop();
    /* PLL zrodlem SYSCLK */
    rcc_sysclk_config(RCC_SYSCLKSource_PLLCLK);
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)0x08);
    /* Zrodlo taktowania USB */
    rcc_otgfs_clk_config(RCC_OTGFSCLKSource_PLLVCO_Div3);
    //Setup NVIC vector at begin of flash
    SCB->VTOR = NVIC_VectTab_FLASH;
}

/* ------------------------------------------------------------------ */
//Setup the systick timer at ISIX_HZ (default 1000HZ)
void timer_setup()
{
	SysTick->LOAD = ISIX_HZ * (HCLK_HZ/(8*MHZ));
	SysTick->CTRL |= CTRL_TICKINT_Set;
	//System counter enable
	SysTick->CTRL |= SysTick_Counter_Enable;
}

/* ------------------------------------------------------------------ */
//! This function is called just before call global constructors
void _external_startup(void)
{

	//Initialize system perhipheral
	uc_periph_setup();

	//1 bit for preemtion priority
	nvic_priority_group(NVIC_PriorityGroup_1);

	//System priorities
	nvic_set_priority(PendSV_IRQn,1,0x7);

	//System priorities
	nvic_set_priority(SVCall_IRQn,1,0x7);

	//Set timer priority
	nvic_set_priority(SysTick_IRQn,1,0x7);

	//Initialize isix
	isix_init(ISIX_NUM_PRIORITIES);

	//Setup the systick timer
	timer_setup();
}

/* ------------------------------------------------------------------ */
