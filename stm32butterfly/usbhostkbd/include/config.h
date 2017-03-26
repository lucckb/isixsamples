/* ------------------------------------------------------------------ */
/*
 * config.hpp
 *
 *  Created on: 2010-01-28
 *      Author: lucck
 */
/* ------------------------------------------------------------------ */
#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_
/* ------------------------------------------------------------------ */
//HCLK system speed
#define  CONFIG_XTAL_HZ   8000000
#define  CONFIG_HCLK_HZ  72000000
#define	 CONFIG_PCLK1_HZ (CONFIG_HCLK_HZ/2)
#define  CONFIG_PCLK2_HZ (CONFIG_HCLK_HZ)
/* ------------------------------------------------------------------ */ 
//!Configure usb stack
#define CONFIG_USBHOST_VBUS_GPIO_N  D
#define CONFIG_USBHOST_VBUS_PIN   14
#define CONFIG_USBHOST_VBUS_ON      1
#define CONFIG_USBHOST_VBUS_OTYPE  GPIO_CNF_GPIO_PP
#define CONFIG_USBLIB_US_TIM_N 2
#define CONFIG_USBHOST_USB_IRQ_MASK_VALUE 1
/* ------------------------------------------------------------------ */

#endif /* CONFIG_HPP_ */
/* ------------------------------------------------------------------ */
