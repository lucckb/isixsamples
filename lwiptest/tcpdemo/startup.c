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
#include "config.h"
/* ------------------------------------------------------------------ */
//! Configure ADC prescaler to 8
#define RCC_CFGR_ADCPRE_8 (3<<14)
//! HSE oscilator control
#define RCC_CR_HSI_ON  (1<<0)
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
	//Configure CLK clock
    RCC->CR &= ~RCC_CR_HSEON;
    //Disable Bypass
    RCC->CR &= ~RCC_CR_HSEBYP;
    //Enable high speed oscilator
    RCC->CR  |= RCC_CR_HSEON;
    //Wait for setup HSE
    while(1)
    {
        if(RCC->CR & RCC_CR_HSERDY) break;
    }
    //Configure flash: Prefetch enable and 1 wait state
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;

    /* Configure system clocks ALL clocks freq 72MHZ APB1 36MHz
    * PLL x 2
     */
    RCC->CFGR = RCC_CFGR_ADCPRE_8  | RCC_CFGR_PPRE1_DIV2 | RCC_PLLMul_9 |
    		RCC_CFGR_PPRE2_DIV1 | RCC_CFGR_PLLSRC;
    RCC->CFGR2 = RCC_CFGR2_PREDIV1_DIV3;

    // At end disable HSI oscilator for power reduction
    RCC->CR &= ~RCC_CR_HSI_ON;

    //Enable PLL
    RCC->CR |= RCC_CR_PLLON;

    //Wait for PLL sync
    while(1)
    {
	    if(RCC->CR & RCC_CR_PLLRDY)
	    {
	        //PLL as system clock
	        RCC->CFGR |=  RCC_CFGR_SW_PLL;
	        break;
	    }
    }
    //rcc_mco_config( RCC_MCO_HSE );
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
