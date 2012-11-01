/* ------------------------------------------------------------------ */
/*
 * example1.c
 *
 * Blinking leds and Joystick with Nokia 3310 display
 * ISIX RTOS C example 1
 *
 *  Created on: 18-09-2010
 *      Author: lucck
 */
/* ------------------------------------------------------------------ */
#include <isix.h>
#include <stm32lib.h>
#include <lwip/init.h>
#include <dbglog.h>
#include <usart_simple.h>
#include "config.h"
#include <stm32rcc.h>
#include <stm32system.h>
#include <stm32gpio.h>
#include <stm32_eth.h>
#include <stdbool.h>
#include "tcpecho/tcpecho.h"
/* ------------------------------------------------------------------ */
//Led Port
#define LED_PORT GPIOE
#define LED_PIN 14

//Blinking time and prio
#define BLINK_TIME 500
#define BLINKING_TASK_PRIO 3

/* ------------------------------------------------------------------ */
/** Blinking led task function */
static ISIX_TASK_FUNC(blinking_task, entry_param)
{
	(void)entry_param;
	RCC->APB2ENR |= RCC_APB2Periph_GPIOE;
	gpio_config(LED_PORT,LED_PIN,GPIO_MODE_10MHZ,GPIO_CNF_GPIO_PP);
	for(;;)
	{
		//Enable LED
		gpio_clr( LED_PORT, LED_PIN );
		//Wait time
		isix_wait_ms( BLINK_TIME );
		//Disable LED
		gpio_set( LED_PORT, LED_PIN );
		//Wait time
		isix_wait_ms( BLINK_TIME );
	}
}


/* ------------------------------------------------------------------ */
/**
  * @brief  Deinitializes the ETHERNET peripheral registers to their default reset values.
  * @param  None
  * @retval None
  */
static void eth_deinit(void)
{

  rcc_ahb_periph_reset_cmd( RCC_AHBPeriph_ETH_MAC, true );
  nop();
  rcc_ahb_periph_reset_cmd( RCC_AHBPeriph_ETH_MAC, false );
  nop();
}
/* ------------------------------------------------------------------ */
#define PHY_ADDRESS       0x01 /* Relative to STM3210C-EVAL Board */
#define MII_MODE

/* ------------------------------------------------------------------ */
#define GPIO_Remap_ETH              ((uint32_t)0x00200020)

//GPIO initialize
static void eth_gpio_init(bool provide_mco)
{
    //Enable gpios
	rcc_apb2_periph_clock_cmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
		RCC_APB2Periph_GPIOC |	RCC_APB2Periph_GPIOD |RCC_APB2Periph_AFIO, true);
	/* ETHERNET pins configuration */
	  /* AF Output Push Pull:
	  - ETH_MII_MDIO / ETH_RMII_MDIO: PA2
	  - ETH_MII_MDC / ETH_RMII_MDC: PC1
	  - ETH_MII_TXD2: PC2
	  - ETH_MII_TX_EN / ETH_RMII_TX_EN: PB11
	  - ETH_MII_TXD0 / ETH_RMII_TXD0: PB12
	  - ETH_MII_TXD1 / ETH_RMII_TXD1: PB13
	  - ETH_MII_PPS_OUT / ETH_RMII_PPS_OUT: PB5
	  - ETH_MII_TXD3: PB8 */

	  /* Configure PA2 as alternate function push-pull */
	  gpio_config( GPIOA, 2,  GPIO_MODE_50MHZ, GPIO_CNF_ALT_PP );
	  /* Configure PC1, PC2 and PC3 as alternate function push-pull */
	  gpio_config_ext( GPIOC, (1<<1)| (1<<2), GPIO_MODE_50MHZ, GPIO_CNF_ALT_PP );
	  /* Configure PB5, PB8, PB11, PB12 and PB13 as alternate function push-pull */
	  gpio_config_ext( GPIOB, (1<<5)|(1<<8)|(1<<11)|(1<<12)|(1<<13), GPIO_MODE_50MHZ, GPIO_CNF_ALT_PP );


	  /**************************************************************/
	  /*               For Remapped Ethernet pins                   */
	  /*************************************************************/
	  /* Input (Reset Value):
	  - ETH_MII_CRS CRS: PA0
	  - ETH_MII_RX_CLK / ETH_RMII_REF_CLK: PA1
	  - ETH_MII_COL: PA3
	  - ETH_MII_RX_DV / ETH_RMII_CRS_DV: PD8
	  - ETH_MII_TX_CLK: PC3
	  - ETH_MII_RXD0 / ETH_RMII_RXD0: PD9
	  - ETH_MII_RXD1 / ETH_RMII_RXD1: PD10
	  - ETH_MII_RXD2: PD11
	  - ETH_MII_RXD3: PD12
	  - ETH_MII_RX_ER: PB10 */

	  /* ETHERNET pins remapp in STM3210C-EVAL board: RX_DV and RxD[3:0] */
	  gpio_pin_remap_config(GPIO_Remap_ETH, true);

	  /* Configure PA0, PA1 and PA3 as input */
	  gpio_config_ext(GPIOA, (1<<0)|(1<<1)|(1<<3), GPIO_MODE_INPUT, GPIO_CNF_IN_FLOAT );

	  /* Configure PB10 as input */
	  gpio_config( GPIOB, 10, GPIO_MODE_INPUT, GPIO_CNF_IN_FLOAT );

	  /* Configure PC3 as input */
	  gpio_config( GPIOC, 3, GPIO_MODE_INPUT, GPIO_CNF_IN_FLOAT );

	  /* Configure PD8, PD9, PD10, PD11 and PD12 as input */
	  gpio_config_ext(GPIOD, (1<<8)|(1<<9)|(1<<10)|(1<<11)|(1<<12), GPIO_MODE_INPUT, GPIO_CNF_IN_FLOAT );

	  /* MCO pin configuration------------------------------------------------- */
	  /* Configure MCO (PA8) as alternate function push-pull */
	  if(provide_mco)
		  gpio_config( GPIOA, 8 , GPIO_MODE_50MHZ, GPIO_CNF_ALT_PP);
}

/* ------------------------------------------------------------------ */
/**
  * @brief  Configures the Ethernet Interface
  * @param  None
  * @retval None
  */
static void ethernet_init(bool provide_mco)
{
    //Enable eth stuff
	rcc_ahb_periph_clock_cmd( RCC_AHBPeriph_ETH_MAC | RCC_AHBPeriph_ETH_MAC_Tx |
			RCC_AHBPeriph_ETH_MAC_Rx, true );

  eth_gpio_init(provide_mco);
  ETH_InitTypeDef ETH_InitStructure;

  /* MII/RMII Media interface selection ------------------------------------------*/
#ifdef MII_MODE /* Mode MII with STM3210C-EVAL  */
  gpio_eth_media_interface_config(GPIO_ETH_MediaInterface_MII);

 if( provide_mco)
 {
  /* Get HSE clock = 25MHz on PA8 pin (MCO) */
 // RCC_MCOConfig(RCC_MCO_HSE);
  rcc_mco_config( RCC_MCO_HSE );
 }
#elif defined RMII_MODE  /* Mode RMII with STM3210C-EVAL */
  GPIO_ETH_MediaInterfaceConfig(GPIO_ETH_MediaInterface_RMII);

  /* Set PLL3 clock output to 50MHz (25MHz /5 *10 =50MHz) */
  RCC_PLL3Config(RCC_PLL3Mul_10);
  /* Enable PLL3 */
  RCC_PLL3Cmd(ENABLE);
  /* Wait till PLL3 is ready */
  while (RCC_GetFlagStatus(RCC_FLAG_PLL3RDY) == RESET)
  {}

  /* Get PLL3 clock on PA8 pin (MCO) */
  RCC_MCOConfig(RCC_MCO_PLL3CLK);
#endif

  /* Reset ETHERNET on AHB Bus */
  eth_deinit();

  /* Software reset */
  ETH_SoftwareReset();

  /* Wait for software reset */
  while (ETH_GetSoftwareResetStatus() == SET) nop();

  /* ETHERNET Configuration ------------------------------------------------------*/
  /* Call ETH_StructInit if you don't like to configure all ETH_InitStructure parameter */
  ETH_StructInit(&ETH_InitStructure);
  /* Fill ETH_InitStructure parametrs */
  /*------------------------   MAC   -----------------------------------*/
  ETH_InitStructure.ETH_AutoNegotiation = ETH_AutoNegotiation_Enable;
  ETH_InitStructure.ETH_LoopbackMode = ETH_LoopbackMode_Disable;
  ETH_InitStructure.ETH_RetryTransmission = ETH_RetryTransmission_Disable;
  ETH_InitStructure.ETH_AutomaticPadCRCStrip = ETH_AutomaticPadCRCStrip_Disable;
  ETH_InitStructure.ETH_ReceiveAll = ETH_ReceiveAll_Disable;
  ETH_InitStructure.ETH_BroadcastFramesReception = ETH_BroadcastFramesReception_Enable;
  ETH_InitStructure.ETH_PromiscuousMode = ETH_PromiscuousMode_Disable;
  ETH_InitStructure.ETH_MulticastFramesFilter = ETH_MulticastFramesFilter_Perfect;
  ETH_InitStructure.ETH_UnicastFramesFilter = ETH_UnicastFramesFilter_Perfect;
#ifdef ISIX_TCPIPLIB_CHECKSUM_BY_HARDWARE
  ETH_InitStructure.ETH_ChecksumOffload = ETH_ChecksumOffload_Enable;
#endif

  /*------------------------   DMA   -----------------------------------*/

  /* When we use the Checksum offload feature, we need to enable the Store and Forward mode:
  the store and forward guarantee that a whole frame is stored in the FIFO, so the MAC can insert/verify the checksum,
  if the checksum is OK the DMA can handle the frame otherwise the frame is dropped */
  ETH_InitStructure.ETH_DropTCPIPChecksumErrorFrame = ETH_DropTCPIPChecksumErrorFrame_Enable;
  ETH_InitStructure.ETH_ReceiveStoreForward = ETH_ReceiveStoreForward_Enable;
  ETH_InitStructure.ETH_TransmitStoreForward = ETH_TransmitStoreForward_Enable;

  ETH_InitStructure.ETH_ForwardErrorFrames = ETH_ForwardErrorFrames_Disable;
  ETH_InitStructure.ETH_ForwardUndersizedGoodFrames = ETH_ForwardUndersizedGoodFrames_Disable;
  ETH_InitStructure.ETH_SecondFrameOperate = ETH_SecondFrameOperate_Enable;
  ETH_InitStructure.ETH_AddressAlignedBeats = ETH_AddressAlignedBeats_Enable;
  ETH_InitStructure.ETH_FixedBurst = ETH_FixedBurst_Enable;
  ETH_InitStructure.ETH_RxDMABurstLength = ETH_RxDMABurstLength_32Beat;
  ETH_InitStructure.ETH_TxDMABurstLength = ETH_TxDMABurstLength_32Beat;
  ETH_InitStructure.ETH_DMAArbitration = ETH_DMAArbitration_RoundRobin_RxTx_2_1;
  /* Configure Ethernet */
  if ( ETH_Init(&ETH_InitStructure, PHY_ADDRESS, HCLK_HZ) == 1 )
  {
	  dbprintf("ETH init success %s"
			  , ETH_InitStructure.ETH_Speed == ETH_Speed_100M?"100MBPS":"10MBPS");
  }
  else
  {
	  dbprintf("ETH init fail mode");
  }
	#define PHYCR     0x19
   #define LED_CNFG0 0x0020
   #define LED_CNFG1 0x0040
   uint16_t phyreg;

   /* konfiguracja diod świecących na ZL3ETH
      zielona - link status:
      on = good link, off = no link, blink = activity
      pomarańczowa - speed:
      on = 100 Mb/s, off = 10 Mb/s */
   phyreg = ETH_ReadPHYRegister( PHY_ADDRESS, PHYCR);
   phyreg &= ~(LED_CNFG0 | LED_CNFG1);
   ETH_WritePHYRegister( PHY_ADDRESS, PHYCR, phyreg);

}

/* ------------------------------------------------------------------ */
void tcp_eth_init(void);


//Initialize the TCPIP library
static void tcpiplib_init()
{
    /*****  Initialize the system stuff  ***************/
	ethernet_init(false);
	tcp_eth_init();
	tcpecho_init();
}

/* ------------------------------------------------------------------ */

//App main entry point
int main(void)
{
	dblog_init( usartsimple_putc, NULL, usartsimple_init,
			USART2,115200,true, PCLK1_HZ, PCLK2_HZ );
	//receive_test();

	//Create ISIX blinking task
	isix_task_create( blinking_task, NULL,
			ISIX_PORT_SCHED_MIN_STACK_DEPTH, BLINKING_TASK_PRIO
	);
	dbprintf("Hello from TCPIP stack example");
	//Initialize the tcpip library
    tcpiplib_init();
	//Start the isix scheduler
	isix_start_scheduler();
}


enum crash_mode
{
	CRASH_TYPE_USER=1,
	CRASH_TYPE_SYSTEM
};
/* ------------------------------------------------------------------ */
static inline void crash_info(enum crash_mode crash_type, unsigned long * SP)
{
	//Disable interrupt
	irq_disable();
	//Initialize usart simple no interrupt
	tiny_printf("\r\n\r\n ^^^^^^^^^^ CPU Crashed in [%s] mode!!! ARMv7m core regs: ^^^^^^^^^\r\n",
			crash_type==CRASH_TYPE_USER?"USER":"SYSTEM" );
	tiny_printf("[R0=%08x]\t[R1=%08x]\t[R2=%08x]\t[R3=%08x]\r\n", SP[0],SP[1],SP[2],SP[3]);
	tiny_printf("[R12=%08x]\t[LR=%08x]\t[PC=%08x]\t[PSR=%08x]\r\n",SP[4],SP[5],SP[6],SP[7]);
	const unsigned long rBFAR = (*((volatile unsigned long *)(0xE000ED38)));
	const unsigned long rCFSR = (*((volatile unsigned long *)(0xE000ED28)));
	const unsigned long rHFSR = (*((volatile unsigned long *)(0xE000ED2C)));
	const unsigned long rDFSR = (*((volatile unsigned long *)(0xE000ED30)));
	const unsigned long rAFSR = (*((volatile unsigned long *)(0xE000ED3C)));
	tiny_printf("[BAFR=%08x]\t[CFSR=%08x]\t[HFSR=%08x]\t[DFSR=%08x]\r\n",rBFAR,rCFSR,rHFSR,rDFSR);
	tiny_printf("[AFSR=%08x]\r\n", rAFSR);
	for(;;) wfi();
}
/* ------------------------------------------------------------------ */
void hard_fault_exception_vector(void) __attribute__((__interrupt__,naked));

/* ------------------------------------------------------------------ */
void hard_fault_exception_vector(void)
{
	unsigned long *sp;
	enum crash_mode cmode;
	//Check for SP or MSP
	asm(
		"TST LR, #4\n"
	    "ITTEE EQ\n"
	    "MRSEQ %[stackptr], MSP\n"
		"MOVEQ %[crashm],%[tsystem]\n"
	    "MRSNE %[stackptr], PSP\n"
		"MOVNE %[crashm],%[tuser]\n"
		: [stackptr] "=r"(sp), [crashm] "=r"(cmode):
		  [tuser]"I"(CRASH_TYPE_USER),[tsystem]"I"(CRASH_TYPE_SYSTEM)
		);
	//Print the crash info
	crash_info( cmode, sp );
}
