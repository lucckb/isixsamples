#include <isix.h>
#include <stm32lib.h>
#include <dbglog.h>
#include <usart_simple.h>
#include "config.hpp"
#include <stdexcept>
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
static void flash_latency(uint32_t frequency)
{
	uint32_t wait_states;

	wait_states = frequency / 30000000ul;	// calculate wait_states (30M is valid for 2.7V to 3.6V voltage range, use 24M for 2.4V to 2.7V, 18M for 2.1V to 2.4V or 16M for  1.8V to 2.1V)
	wait_states &= 7;						// trim to max allowed value - 7

	FLASH->ACR = wait_states;				// set wait_states, disable all caches and prefetch
	FLASH->ACR = FLASH_ACR_DCRST | FLASH_ACR_ICRST | wait_states;	// reset caches
	FLASH->ACR = FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_PRFTEN | wait_states;	// enable caches and prefetch
}
/* ------------------------------------------------------------------ */
static void fpu_enable(void)
{
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
	SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));	// set CP10 and CP11 Full Access
#endif
}


#define RCC_PLLCFGR_PLLM_bit                            0
#define RCC_PLLCFGR_PLLN_bit                            6
#define RCC_PLLCFGR_PLLP_bit                            16
#define RCC_PLLCFGR_PLLQ_DIV9_value                     9
#define RCC_PLLCFGR_PLLQ_DIV9                           (RCC_PLLCFGR_PLLQ_DIV9_value << RCC_PLLCFGR_PLLQ_bit)
#define RCC_PLLCFGR_PLLQ_bit                            24
/* ------------------------------------------------------------------ */
static uint32_t pll_start(uint32_t crystal, uint32_t frequency)
{
	uint32_t div, mul, div_core, vco_input_frequency, vco_output_frequency, frequency_core;
	uint32_t best_div = 0, best_mul = 0, best_div_core = 0, best_frequency_core = 0;

	RCC->CR  |= RCC_CR_HSEON;
	flash_latency(frequency);				// configure Flash latency for desired frequency

	for (div = 2; div <= 63; div++)			// PLLM in [2; 63]
	{
		vco_input_frequency = crystal / div;

		if ((vco_input_frequency < 1000000ul) || (vco_input_frequency > 2000000))	// skip invalid settings
			continue;

		for (mul = 64; mul <= 432; mul++)	// PLLN in [64; 432]
		{
			vco_output_frequency = vco_input_frequency * mul;

			if ((vco_output_frequency < 64000000ul) || (vco_output_frequency > 432000000ul))	// skip invalid settings
				continue;

			for (div_core = 2; div_core <= 8; div_core += 2)	// PLLP in {2, 4, 6, 8}
			{
				frequency_core = vco_output_frequency / div_core;

				if (frequency_core > frequency)	// skip values over desired frequency
					continue;

				if (frequency_core > best_frequency_core)	// is this configuration better than previous one?
				{
					best_frequency_core = frequency_core;	// yes - save values
					best_div = div;
					best_mul = mul;
					best_div_core = div_core;
				}
			}
		}
	}

	RCC->PLLCFGR = (best_div << RCC_PLLCFGR_PLLM_bit) | (best_mul << RCC_PLLCFGR_PLLN_bit) | ((best_div_core / 2 - 1) << RCC_PLLCFGR_PLLP_bit) | RCC_PLLCFGR_PLLQ_DIV9 | RCC_PLLCFGR_PLLSRC_HSE;	// configure PLL factors, always divide USB clock by 9

	RCC->CFGR = RCC_CFGR_PPRE2_DIV2 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_HPRE_DIV1;	// AHB - no prescaler, APB1 - divide by 4, APB2 - divide by 2

	while(1)
	{
	   if(RCC->CR & RCC_CR_HSERDY) break;
	}
	RCC->CR |= RCC_CR_PLLON;
	while(1)
	{
		if(RCC->CR & RCC_CR_PLLRDY) break;
	}

	RCC->CFGR |= RCC_CFGR_SW_PLL;			// change SYSCLK to PLL
	while (((RCC->CFGR) & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);	// wait for switch
	return best_frequency_core;
}
#else
#error Selected MCU type is not supported by this sample
#endif
/* ------------------------------------------------------------------ */
//Peripheral setup
void uc_periph_setup()
{
	fpu_enable();
	pll_start(8000000ul, 168000000ul);
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
		//RCC->APB2ENR |= RCC_APB2Periph_GPIOE;
		//io_config(LED_PORT,LED_PIN,GPIO_MODE_10MHZ,GPIO_CNF_GPIO_PP);
	}
protected:
	//Main function
	virtual void main()
	{
		while(true)
		{
			//Enable LED
			//stm32::io_clr( LED_PORT, LED_PIN );
			//Wait time
			//isix::isix_wait( isix::isix_ms2tick(BLINK_TIME) );
			//Disable LED
		//	stm32::io_set( LED_PORT, LED_PIN );
			//Wait time
			//isix::isix_wait( isix::isix_ms2tick(BLINK_TIME) );
		}
	}
private:
	static const unsigned STACK_SIZE = 256;
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
//		RCC->APB2ENR |= RCC_APB2Periph_GPIOE;
//		io_config(LED_PORT,LED_PIN,GPIO_MODE_10MHZ,GPIO_CNF_GPIO_PP);
//		io_config(LED_PORT,NOTIFY_PIN,GPIO_MODE_10MHZ,GPIO_CNF_GPIO_PP);
	}
protected:
	//Main function
	virtual void main()
	{
		//Last key state
		bool state = true;
		//Task/thread main loop
		try
		{
			while(true)
			{
				execute_keycheck(state);
			}
		}
		catch( int &val)
		{
//			stm32::io_clr( LED_PORT,NOTIFY_PIN );
			dbprintf("INT exception [%d]", val);
		}
		catch( const std::exception &e )
		{
//			stm32::io_clr( LED_PORT,NOTIFY_PIN );
			dbprintf("std::exception [%s]", e.what());
		}
	}
private:
	//Execute keycheck function
	void execute_keycheck(bool &p_state)
	{

		//Change state on rising edge
//		if(stm32::io_get(KEY_PORT, KEY_PIN) && !p_state)
		{
			is_enabled = !is_enabled;
		}
		//Get previous state
//		p_state = stm32::io_get(KEY_PORT, KEY_PIN);
		//If enabled change state
//		if(is_enabled) stm32::io_clr( LED_PORT, LED_PIN );
//		else stm32::io_set( LED_PORT, LED_PIN );
		//Wait short time
		isix::isix_wait( isix::isix_ms2tick(DELAY_TIME) );
//		if( !stm32::io_get(KEY_PORT, KEY_RAISE_LOGIC ))
		{
			/** From raise to catch 151us **/
//			stm32::io_set( LED_PORT,NOTIFY_PIN );
			throw(std::logic_error("critical error raised"));
		}
//		if( !stm32::io_get(KEY_PORT, KEY_RAISE_INT ))
		{
			/** From raise to catch 108us **/
//			stm32::io_set( LED_PORT,NOTIFY_PIN );
			throw(-1);
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

