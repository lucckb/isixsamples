
/*
 * config.hpp
 *
 *  Created on: 2010-01-28
 *      Author: lucck
 */

#pragma once

//Enable isix shutdown API
#define CONFIG_ISIX_SHUTDOWN_API 1

//HCLK system speed
#define  CONFIG_XTAL_HZ   8000000
#define  CONFIG_HCLK_HZ  120000000
#define	 CONFIG_PCLK1_HZ (CONFIG_HCLK_HZ/4)
#define  CONFIG_PCLK2_HZ (CONFIG_HCLK_HZ/2)

//!Configure usb stack
#define CONFIG_USBHOST_VBUS_GPIO_N  B
#define CONFIG_USBHOST_VBUS_PIN   15
#define CONFIG_USBHOST_VBUS_ON      0
#define CONFIG_USBHOST_VBUS_OTYPE GPIO_OTYPE_OD
#define CONFIG_USBLIB_US_TIM_N 4
#define CONFIG_USBHOST_USB_IRQ_MASK_VALUE 1

//! PSK application board specific config
#define CONFIG_LIBPSK_PTT_PORT  GPIOA
#define CONFIG_LIBPSK_PTT_PIN 3

//! Define SDCARD clk speed  clock drivers etc
#define CONFIG_SDIO_HOST_CLK_KHZ 24000
#define CONFIG_SD_DET_PORT GPIOB
#define CONFIG_SD_DET_PIN 12
#define CONFIG_SDDRV_DMA_STREAM_NO 6
#define CONFIG_SDDRV_INIT_DMA_GPIO_CLKS 0

