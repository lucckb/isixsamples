/*
 * =====================================================================================
 *
 *       Filename:  config.h
 *
 *    Description:  Nixie CLOCK application config
 *
 *        Version:  1.0
 *        Created:  08.05.2016 18:36:36
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Lucjan Bryndza (LB), lbryndza.p(at)boff(dot)pl
 *   Organization:  BoFF
 *
 * =====================================================================================
 */
#pragma once
 
// --------  Hardware specific config ------------
#define  CONFIG_XTAL_HZ   25000000LU
#define  CONFIG_HCLK_HZ  168000000LU
#define	 CONFIG_PCLK1_HZ (CONFIG_HCLK_HZ/4)
#define  CONFIG_PCLK2_HZ (CONFIG_HCLK_HZ/2)

// --------  ISIX specific config ------------
//! Enable poweroff API  
//#define ISIX_CONFIG_SHUTDOWN_API
//! In F103 devs MPU is not present
#define ISIX_CONFIG_MEMORY_PROTECTION_MODEL ISIX_MPROT_LITE
#define ISIX_CONFIG_TASK_STACK_CHECK 1
//! Define log level for application
#define FOUNDATION_CONFIG_LOGLEVEL FOUNDATION_DBGLOG_DEBUG
#define ISIX_CONFIG_NUM_PRIORITIES 4

// ---- Ethernet specific config ----
#define ETH_DRV_USE_RMII  1
#define PHY_INT_USE_INTERRUPT 0
#define PHY_INT_EXTI_LINE_IRQ_N EXTI0_IRQn
#define PHY_INT_GPIO_PORT A
#define PHY_INT_EXTI_NUM 0
#define ETH_DRV_PHY_ADDR 7
