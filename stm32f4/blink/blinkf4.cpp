#include <isix.h>
#include <stm32lib.h>
#include <dbglog.h>
#include <usart_simple.h>
#include "config.hpp"
#include <stm32system.h>
#include <stm32rcc.h>
/* ------------------------------------------------------------------ */
namespace {
/* ------------------------------------------------------------------ */
//! Configure ADC prescaler to 8
const unsigned long RCC_CFGR_ADCPRE_8 = (3<<14);

//! HSE oscilator control
const unsigned long RCC_CR_HSI_ON = (1<<0);
//Number of isix threads

const unsigned ISIX_NUM_PRIORITIES = 4;

//SysTimer values
const unsigned MHZ = 1000000;
const unsigned CTRL_TICKINT_Set = 2;
const unsigned SysTick_Counter_Enable = 1;


/* ------------------------------------------------------------------ */
/** Cortex stm32 System setup
 * Clock and flash configuration for selected rate
 */
#if defined(STM32MCU_MAJOR_TYPE_F1)
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

    /* Configure system clocks ALL clocks freq 16MHz
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
    //Setup NVIC vector at begin of flash
    SCB->VTOR = NVIC_VectTab_FLASH;
}
#elif defined(STM32MCU_MAJOR_TYPE_F4)

/* ------------------------------------------------------------------ */
#else
#error Selected MCU type is not supported by this sample
#endif
/* ------------------------------------------------------------------ */
//Peripheral setup
void uc_periph_setup()
{
	stm32::rcc_flash_latency( config::HCLK_HZ );
	stm32::rcc_pll1_sysclk_setup( stm32::e_sysclk_hse_pll, config::XTAL_HZ , config::HCLK_HZ );
	//Setup dividers
	RCC->CFGR &= ~(RCC_CFGR_PPRE2 | RCC_CFGR_PPRE1 );
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV2 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_HPRE_DIV1;
	//Setup NVIC vector at begin of flash
	SCB->VTOR = NVIC_VectTab_FLASH;
}

/* ------------------------------------------------------------------ */
//Setup the systick timer at ISIX_HZ (default 1000HZ)
void timer_setup()
{
	SysTick->LOAD = isix::ISIX_HZ * (config::HCLK_HZ/(8*MHZ));
	SysTick->CTRL |= CTRL_TICKINT_Set;
	//System counter enable
	SysTick->CTRL |= SysTick_Counter_Enable;
}

/* ------------------------------------------------------------------ */
extern "C"
{

/* ------------------------------------------------------------------ */
//! This function is called just before call global constructors
void _external_startup(void)
{

	//Initialize system perhipheral
	uc_periph_setup();

	//1 bit for preemtion priority
	stm32::nvic_priority_group(NVIC_PriorityGroup_1);

	//System priorities
	stm32::nvic_set_priority(PendSV_IRQn,1,0x7);

	//System priorities
	stm32::nvic_set_priority(SVCall_IRQn,1,0x7);

	//Set timer priority
	stm32::nvic_set_priority(SysTick_IRQn,1,0x7);

	//Initialize isix
	isix::isix_init(ISIX_NUM_PRIORITIES);

	//Setup the systick timer
	timer_setup();
}
/* ------------------------------------------------------------------ */
} /* extern C */

/* ------------------------------------------------------------------ */
}
/* ------------------------------------------------------------------ */
namespace app
{

/* ------------------------------------------------------------------ */
class ledblink: public isix::task_base
{
public:
	//Constructor
	ledblink() : task_base(STACK_SIZE,TASK_PRIO), LED_PORT(GPIOE)
	{
		using namespace stm32;
		//Enable PE in APB2
		RCC->AHB1ENR |= RCC_AHB1Periph_GPIOE;
		//gpio_config(LED_PORT,LED_PIN,GPIO_MODE_10MHZ,GPIO_CNF_GPIO_PP);
		gpio_config(LED_PORT, LED_PIN, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_SPEED_25MHZ, GPIO_OTYPE_PP );
	}
protected:
	//Main function
	virtual void main()
	{
		volatile float ala_z = 1.2;
		while(true)
		{
			//Enable LED
			stm32::gpio_clr( LED_PORT, LED_PIN );
			//Wait time
			isix::isix_wait( isix::isix_ms2tick(BLINK_TIME) );
			//Disable LED
			stm32::gpio_set( LED_PORT, LED_PIN );
			//Wait time
			isix::isix_wait( isix::isix_ms2tick(BLINK_TIME) );
			ala_z *= 1.1;
			dbprintf("Ala value=%d", (int)ala_z);
		}
	}
private:
	static const unsigned STACK_SIZE = 2048;
	static const unsigned TASK_PRIO = 3;
	GPIO_TypeDef * const LED_PORT;
	static const unsigned LED_PIN = 14;
	static const unsigned BLINK_TIME = 500;
};
/* ------------------------------------------------------------------ */

class ledkey: public isix::task_base
{
public:
	//Constructor
	ledkey()
		: task_base(STACK_SIZE,TASK_PRIO), is_enabled(false),
		  KEY_PORT(GPIOE), LED_PORT(GPIOE)
	{
		using namespace stm32;
		//Enable PE in APB2
		RCC->AHB1ENR |= RCC_AHB1Periph_GPIOE;
		gpio_config(LED_PORT, LED_PIN, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_SPEED_25MHZ, GPIO_OTYPE_PP );
		gpio_config(LED_PORT, NOTIFY_PIN, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_SPEED_25MHZ, GPIO_OTYPE_PP );
	}
protected:
	//Main function
	virtual void main()
	{
		volatile float ala_j = 1.2;
		for(;;)
		{
			isix::isix_wait( isix::isix_ms2tick(600) );
			ala_j += 50.0;
			dbprintf("Ala2 value=%d", (int)ala_j);
		}
	}
private:
		static const unsigned STACK_SIZE = 2048;
		static const unsigned TASK_PRIO = 3;
		bool is_enabled;
		GPIO_TypeDef * const KEY_PORT;
		GPIO_TypeDef * const LED_PORT;
		static const unsigned LED_PIN = 15;
		static const unsigned KEY_PIN = 8;
		static const unsigned NOTIFY_PIN = 0;
		static const unsigned KEY_RAISE_LOGIC = 9;
		static const unsigned KEY_RAISE_INT = 10;
		static const unsigned DELAY_TIME = 25;
};

/* ------------------------------------------------------------------ */

}
/* ------------------------------------------------------------------ */
//App main entry point
int main()
{
	 dblog_init( stm32::usartsimple_putc, NULL, stm32::usartsimple_init,
	    		USART2,115200,true, config::PCLK1_HZ, config::PCLK2_HZ );
	 dbprintf(" Exception presentation app using ISIXRTOS ");

	//The blinker class
	static app::ledblink led_blinker;
	//The ledkey class
	static app::ledkey led_key;
	//Start the isix scheduler
	isix::isix_start_scheduler();
}

/* ------------------------------------------------------------------ */

