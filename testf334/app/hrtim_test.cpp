/*
 * hrtim_test.cpp
 *
 *  Created on: 6 sty 2018
 *      Author: lucck
 */


#if 0
#include <stm32_ll_hrtim.h>
#include <stm32rcc.h>
#include <stm32gpio.h>
#endif
#include <isix.h>
#include <foundation/sys/dbglog.h>
#include <stm32f3xx_ll_hrtim.h>

namespace app {

#if 0
static constexpr uint16_t BUCK_PWM_PERIOD = 18432;
static constexpr uint16_t DT_FALLING = 230;
static constexpr uint16_t DT_RISING = 230;
#endif

void hrtim_test_init()
{
#if 0
	using namespace stm32;
	//Temporary enable GPIO on
	gpio_config(GPIOC, 9, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, AGPIO_SPEED_LOW, 0);
	gpio_set(GPIOC,9);

	//GPIO configuration
	gpio_config(GPIOA, 8, GPIO_MODE_ALTERNATE, GPIO_PUPD_NONE, AGPIO_SPEED_FULL, 0);
	gpio_config(GPIOA, 9, GPIO_MODE_ALTERNATE, GPIO_PUPD_NONE, AGPIO_SPEED_FULL, 0);
	gpio_pin_AF_config( GPIOA, 8, GPIO_AF_13 );
	gpio_pin_AF_config( GPIOA, 9, GPIO_AF_13 );
	rcc_hrtim1_clk_config( RCC_HRTIM1CLK_PLLCLK );
	rcc_apb2_periph_clock_cmd(RCC_APB2ENR_HRTIM1, true);
	LL_HRTIM_ConfigDLLCalibration(HRTIM1,
	                                LL_HRTIM_DLLCALIBRATION_MODE_CONTINUOUS,
	                                LL_HRTIM_DLLCALIBRATION_RATE_14);
	//Wait for flag activation
	for(int tout=0; LL_HRTIM_IsActiveFlag_DLLRDY(HRTIM1)==RESET&&tout<50;++tout ) {
		isix::wait_ms(1);
	}
	dbg_debug( "HRTIM activation flag %i", LL_HRTIM_IsActiveFlag_DLLRDY(HRTIM1));
	  /* TIMER A - 250kHz switching frequency */
	  LL_HRTIM_TIM_SetPeriod(HRTIM1,
	                         LL_HRTIM_TIMER_A,
	                         BUCK_PWM_PERIOD);
	  /* TIMER A - set clock prescaling ratio to acheive the highest possible resolution */
	  LL_HRTIM_TIM_SetPrescaler(HRTIM1,
	                            LL_HRTIM_TIMER_A,
	                            LL_HRTIM_PRESCALERRATIO_MUL32);

	  /* TIMER A - Operates in continuous (free-running) mode */
	  LL_HRTIM_TIM_SetCounterMode(HRTIM1,
	                              LL_HRTIM_TIMER_A,
	                              LL_HRTIM_MODE_CONTINUOUS);

	  /* TIMER A - Enable preload mechanism*/
	   LL_HRTIM_TIM_EnablePreload(HRTIM1,
	                              LL_HRTIM_TIMER_A);

	   /* TIMER A - Update on repetition is enabled */
	   LL_HRTIM_TIM_SetUpdateTrig(HRTIM1,
	                              LL_HRTIM_TIMER_A,
	                              LL_HRTIM_UPDATETRIG_REPETITION);

	   /* TIMER A - Dead time insertion is enabled */
	   LL_HRTIM_TIM_EnableDeadTime(HRTIM1,
	                               LL_HRTIM_TIMER_A);
	   /* TIMER A - Configure compare unit 1 (Start with minimal duty cycle) */
	   LL_HRTIM_TIM_SetCompare1(HRTIM1,
	                            LL_HRTIM_TIMER_A,
	                            BUCK_PWM_PERIOD - BUCK_PWM_PERIOD/2);

	   /*## TA1 configuration #####################################################*/

	   /* TA1 - Timer A CMP1 event forces TA1 to its active state */
	   LL_HRTIM_OUT_SetOutputSetSrc(HRTIM1,
	                                LL_HRTIM_OUTPUT_TA1,
	                                LL_HRTIM_CROSSBAR_TIMCMP1);

	   /* TA1 - Timer A period event forces TA1 to its inactive state */
	   LL_HRTIM_OUT_SetOutputResetSrc(HRTIM1,
	                                  LL_HRTIM_OUTPUT_TA1,
	                                  LL_HRTIM_CROSSBAR_TIMPER);

	   /* TA1 - After a fault event TA1 is set to its inactive state */
	   LL_HRTIM_OUT_SetFaultState(HRTIM1,
	                              LL_HRTIM_OUTPUT_TA1,
	                              LL_HRTIM_OUT_FAULTSTATE_INACTIVE);


	   /*## TA2 configuration #####################################################*/

	   /* TA2 - Timer A CMP1 event forces TA1 to its active state */
	   LL_HRTIM_OUT_SetOutputSetSrc(HRTIM1,
	                                LL_HRTIM_OUTPUT_TA2,
	                                LL_HRTIM_CROSSBAR_TIMCMP1);

	   /* TA2 - Timer A period event forces TA1 to its inactive state */
	   LL_HRTIM_OUT_SetOutputResetSrc(HRTIM1,
	                                  LL_HRTIM_OUTPUT_TA2,
	                                  LL_HRTIM_CROSSBAR_TIMPER);

	   /* TA2 - After a fault event TA1 is set to its inactive state */
	   LL_HRTIM_OUT_SetFaultState(HRTIM1,
	                              LL_HRTIM_OUTPUT_TA2,
	                              LL_HRTIM_OUT_FAULTSTATE_INACTIVE);
	   /*## Deadtime insertion configuration #####################################*/

	   /* TIMER A - Deadtime insertion configuration */
	   LL_HRTIM_DT_SetRisingValue(HRTIM1, LL_HRTIM_TIMER_A, DT_RISING);
	   LL_HRTIM_DT_SetFallingValue(HRTIM1, LL_HRTIM_TIMER_A, DT_FALLING);
	   LL_HRTIM_DT_LockRisingSign(HRTIM1, LL_HRTIM_TIMER_A);
	   LL_HRTIM_DT_LockFallingSign(HRTIM1, LL_HRTIM_TIMER_A);

	   /*## ADC trigger configuration #############################################*/

	     /* TIMER A - Configure compare unit 2 (CMP2 event used as ADC trigger) */
	     LL_HRTIM_TIM_SetCompare2(HRTIM1,
	                              LL_HRTIM_TIMER_A,
	                              BUCK_PWM_PERIOD/2);

	     /* ADC trigger initialization */
	     LL_HRTIM_ConfigADCTrig(HRTIM1,
	                            LL_HRTIM_ADCTRIG_2,
	                            LL_HRTIM_ADCTRIG_UPDATE_TIMER_A,
	                            LL_HRTIM_ADCTRIG_SRC24_TIMACMP2);

	     /* HRTIM start-up */
	     /* Enable HRTIM's outputs TA1, TA2, TB1 and TB2 */
	     LL_HRTIM_EnableOutput(HRTIM1,
	                           LL_HRTIM_OUTPUT_TA1 |
	                           LL_HRTIM_OUTPUT_TA2 );

	     /* Start HRTIM's TIMER A and B */
	     LL_HRTIM_TIM_CounterEnable(HRTIM1, LL_HRTIM_TIMER_A );
#endif
}

}




