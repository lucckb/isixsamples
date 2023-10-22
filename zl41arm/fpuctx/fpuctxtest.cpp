#include <isix.h>
#include <stm32lib.h>
#include <foundation/sys/dbglog.h>
#include <usart_simple.h>
#include "config.hpp"
#include <stm32system.h>
#include <stm32rcc.h>
#include <stm32gpio.h>
#include <stm32tim.h>
#include <stm32adc.h>
#include <stm32dma.h>
#include <stm32pwr.h>
#include <stm32crashinfo.h>
#include <isix/arch/irq.h>
 
namespace {
 

//Number of isix threads
const unsigned ISIX_NUM_PRIORITIES = 4;
//SysTimer values
const unsigned MHZ = 1000000;

 
/** Cortex stm32 System setup
 * Clock and flash configuration for selected rate
 */
void uc_periph_setup()
{
    stm32::rcc_flash_latency( config::HCLK_HZ );
    stm32::rcc_pll1_sysclk_setup( stm32::e_sysclk_hse_pll, config::XTAL_HZ , config::HCLK_HZ );
    stm32::rcc_pclk2_config(  RCC_HCLK_Div2 );
    stm32::rcc_pclk1_config(  RCC_HCLK_Div4 );
    //Setup NVIC vector at begin of flash
    SCB->VTOR = NVIC_VectTab_FLASH;
}

 
extern "C"
{
	//Save all fpu regs inc initial by one
	void fputest_fill_and_add( int start );
	// Test context checking after adding ones
	int fputest_fill_and_add_check(int start );
	// Fill basics regs from irq
	void fpuirq_base_regs_fill( int start );
	//Check basic regs filling from irq
	int fpuirq_base_regs_check( int start );

 
//! This function is called just before call global constructors
void _external_startup(void)
{

	//Initialize system perhipheral
	uc_periph_setup();

	//1 bit for preemtion priority
	isix_set_irq_priority_group( isix_cortexm_group_pri7 );

	//Initialize isix
	isix::init(config::HCLK_HZ);

}
 
} /* extern C */

 
}
 
namespace app
{

struct ledblink
{
	ledblink() : LED_PORT(GPIOE)
	{
		using namespace stm32;
		gpio_clock_enable( LED_PORT, true);
		gpio_abstract_config(LED_PORT, LED_PIN, AGPIO_MODE_OUTPUT_PP, AGPIO_SPEED_HALF );
	}
	void main() noexcept
	{
		volatile float ala_z = 1.0;
		while(true)
		{
			//Enable LED
			stm32::gpio_clr( LED_PORT, LED_PIN );
			//Wait time
			isix::wait_ms( BLINK_TIME );
			//Disable LED
			stm32::gpio_set( LED_PORT, LED_PIN );
			//Wait time
			isix::wait_ms( BLINK_TIME );
			ala_z += 1.0;
			dbprintf("VAR1=%d", (int)ala_z);
		}
	}
private:
	GPIO_TypeDef * const LED_PORT;
	static const unsigned LED_PIN = 14;
	static const unsigned BLINK_TIME = 500;
};
 
namespace {
    //! Interrupt FPU context on the lower possible level
    void prepare_fpu_irq_tests() {
        using namespace stm32;
        rcc_apb1_periph_clock_cmd( RCC_APB1Periph_TIM3, true );
        nvic_set_priority( TIM3_IRQn , 1, 1 );
        tim_timebase_init( TIM3, 0, TIM_CounterMode_Up, 65535, 0, 0 );
        tim_it_config( TIM3, TIM_IT_Update, true );
        tim_cmd( TIM3, true );
        nvic_irq_enable( TIM3_IRQn, true );
        nvic_set_priority( TIM2_IRQn, 0, 7 );
        nvic_irq_enable( TIM2_IRQn, true );
    }
    volatile int check_irq_failed = 0;

 //Interrrupt handlers
 extern "C" {
    void tim3_isr_vector() {
       stm32::tim_clear_it_pending_bit( TIM3, TIM_IT_Update );
       fpuirq_base_regs_fill( 55 );
       stm32::nvic_irq_set_pending(  TIM2_IRQn );
       const int irqcr = fpuirq_base_regs_check( 55 );
       if( irqcr ) {
            check_irq_failed = irqcr;
       }
    }
    void tim2_isr_vector() {
         fpuirq_base_regs_fill( 100 );
            if( fpuirq_base_regs_check( 100 ) ) {
            	check_irq_failed = true;
        }
    }
}}
 
struct math_task
{
	math_task(int begin ) : m_begin( begin ) {}
	~math_task() { dbprintf("Math task finished"); }

	void main() noexcept
	{
        fputest_fill_and_add(m_begin);
        for(unsigned it=0;;++it)
		{
            const int fret =  fputest_fill_and_add_check(m_begin);
            if( fret ) {
                dbprintf("Assert failed at %i in %i iteration %i", fret, m_begin, it );
                isix::shutdown_scheduler();
                break;
            }
            if( check_irq_failed ) {
            	 dbprintf("Assert IRQFAILED %i", check_irq_failed );
            	  isix::shutdown_scheduler();
            	 break;
            }
		}
	}
private:
	int m_begin;
};


 

}	//namespace app end
 
namespace {
#ifdef PDEBUG
    isix::semaphore usem(1, 1);
    void usart_lock() {
        usem.wait(ISIX_TIME_INFINITE);
    }
    void usart_unlock() {
        usem.signal();
    }
#endif
}
 
//App main entry point
int main()
{
#ifdef PDEBUG
    stm32::usartsimple_init( USART2,115200,true, config::PCLK1_HZ, config::PCLK2_HZ );
#endif
    dblog_init_putc_locked( stm32::usartsimple_putc, NULL, usart_lock, usart_unlock );
	 dbprintf("FPU context save and restore test ISIXRTOS ");

	static app::ledblink led_blinker;
	static isix::thread blinker_thr = isix::thread_create_and_run(
		2048, 3, isix_task_flag_newlib, &app::ledblink::main, &led_blinker);

	static app::math_task t1(10);
	static isix::thread math1_thr = isix::thread_create_and_run(
		2048, 3, isix_task_flag_newlib, &app::math_task::main, &t1);

	static app::math_task t2(20);
	static isix::thread math2_thr = isix::thread_create_and_run(
		2048, 3, isix_task_flag_newlib, &app::math_task::main, &t2);

    static app::math_task t3(30);
	static isix::thread math3_thr = isix::thread_create_and_run(
		2048, 3, isix_task_flag_newlib, &app::math_task::main, &t3);

    static app::math_task t4(40);
	static isix::thread math4_thr = isix::thread_create_and_run(
		2048, 3, isix_task_flag_newlib, &app::math_task::main, &t4);

    static app::math_task t5(50);
	static isix::thread math5_thr = isix::thread_create_and_run(
		2048, 3, isix_task_flag_newlib, &app::math_task::main, &t5);

    static app::math_task t6(60);
	static isix::thread math6_thr = isix::thread_create_and_run(
		2048, 3, isix_task_flag_newlib, &app::math_task::main, &t6);

    //Simulate failure disable stack preserve
    //FPU->FPCCR &= ~( (1<<31U)|(1<<30));
	//Start the isix scheduler
    app::prepare_fpu_irq_tests();
	isix::start_scheduler();
}
