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
#define  CONFIG_XTAL_HZ   8000000LU
#define  CONFIG_HCLK_HZ  168000000LU
#define	 CONFIG_PCLK1_HZ (CONFIG_HCLK_HZ/4)
#define  CONFIG_PCLK2_HZ (CONFIG_HCLK_HZ/2)

// --------  ISIX specific config ------------
//! Enable poweroff API
//#define CONFIG_ISIX_SHUTDOWN_API 1
#define CONFIG_ISIX_TASK_STACK_CHECK 1
//! Define log level for application
#define CONFIG_FOUNDATION_LOGLEVEL FOUNDATION_DBGLOG_DEBUG


