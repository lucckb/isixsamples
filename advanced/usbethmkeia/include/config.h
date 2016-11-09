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
//Default phy address
//Phy driver name
#define ETH_PHY_DRIVER_NAME _ethernetif_phy_dp83848_drv_
#define ETH_DRV_PHY_ADDR 1
//PHI interrupt pin number
#define PHY_INT_EXTI_NUM 0
//Exti port
#define PHY_INT_GPIO_PORT C
/* ------------------------------------------------------------------ */
//USB configuration
//TX and RX packet number of packet buffers (x64)
#define USBD_ISIX_CDCSERIAL_PACKET_TX_BUF_NBUFS			4
#define USBD_ISIX_CDCSERIAL_PACKET_RX_BUF_NBUFS			8
//USB VID and USB PID
#define USBD_VID                        0x0483
#define USBD_PID                        0x5740
//Description strings
#define USBD_MANUFACTURER_STRING        "BoFF"
#define USBD_PRODUCT_FS_STRING          "ISIX Virtual ComPort"
#define USBD_SERIALNUMBER_FS_STRING     "00000000050C"
#define USBD_CONFIGURATION_FS_STRING    "VCP Config"
#define USBD_INTERFACE_FS_STRING        "VCP Interface"
/* ------------------------------------------------------------------ */
#endif /* CONFIG_H_ */
/* ------------------------------------------------------------------ */
