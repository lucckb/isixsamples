/*
 * i2c_master.c
 *
 *  Created on: 19-09-2010
 *      Author: lucck
 */
/* ------------------------------------------------------------------ */
#include "config.h"
#include "i2c_master.h"
#include <system.h>
#include <isix.h>

/* ------------------------------------------------------------------ */
//I2C PINS
#define I2C1_PORT  GPIOB
#define  I2C1_SDA_PIN 7
#define  I2C1_SCL_PIN  6
#define  I2C1_GPIO_ENR  RCC_APB2Periph_GPIOB
#define  I2C1_ENR      RCC_APB1Periph_I2C1

#define  CR1_PE_RESET  0xFFFE
#define  CR2_FREQ_RESET  0xFFC0
#define  CCR_CCR_SET  0x0FFF
	/* I2C F/S mask */
#define  CCR_FS_SET  0x8000
#define  CR1_CLEAR_MASK  0xFBF5
#define  CR1_SWRST  (1<<15)
#define  I2C_MODE_I2C  0x0000
#define  I2C_ACK_ENABLE  0x0400
#define  I2C_AcknowledgedAddress_7bit  0x4000
#define  I2C_BUS_RW_BIT  0x01

//I2c status
enum {
		//EV1
		I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED  = 0x00060082, /* TRA, BUSY, TXE and ADDR flags */
		I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED = 0x00020002, 		/* BUSY and ADDR flags */
		I2C_EVENT_SLAVE_TRANSMITTER_SECONDADDRESS_MATCHED = 0x00860080,  /* DUALF, TRA, BUSY and TXE flags */
		I2C_EVENT_SLAVE_RECEIVER_SECONDADDRESS_MATCHED =   0x00820000,  /* DUALF and BUSY flags */
		I2C_EVENT_SLAVE_GENERALCALLADDRESS_MATCHED =      0x00120000,  /* GENCALL and BUSY flags */
		//EV2
		I2C_EVENT_SLAVE_BYTE_RECEIVED = 0x00020040,  	/* BUSY and RXNE flags */
		//EV3
		I2C_EVENT_SLAVE_BYTE_TRANSMITTED = 0x00060084,  /* TRA, BUSY, TXE and BTF flags */
		//EV4
		I2C_EVENT_SLAVE_STOP_DETECTED  = 0x00000010, 	 /* STOPF flag */
		//EV5
		I2C_EVENT_MASTER_MODE_SELECT = 0x00030001,   /* BUSY, MSL and SB flag */
		//EV6
		I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED = 0x00070082,  /* BUSY, MSL, ADDR, TXE and TRA flags */
		I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED =  0x00030002,   /* BUSY, MSL and ADDR flags */
		//EV7
		I2C_EVENT_MASTER_BYTE_RECEIVED = 0x00030044,  /* BUSY, MSL and RXNE flags */
		I2C_EVENT_MASTER_BYTE_TRANSMITTING =  0x00070080, /* TRA, BUSY, MSL, TXE flags */
		//EV8_2
		I2C_EVENT_MASTER_BYTE_TRANSMITTED =   0x00070084,  /* TRA, BUSY, MSL, TXE and BTF flags */
		//EV9
		I2C_EVENT_MASTER_MODE_ADDRESS10 =  0x00030008,  /* BUSY, MSL and ADD10 flags */
		//EV3_2
		I2C_EVENT_SLAVE_ACK_FAILURE  = 0x00000400  /* AF flag */
};

//Event error mask
#define EVENT_ERROR_MASK 0xff00
#define  TRANSFER_TIMEOUT  1000
#define  IRQ_PRIO  1
#define  IRQ_SUB 7
//Rest of the data
#define  CR1_ACK_BIT 0x0400
#define  CR1_START_BIT  0x0100
#define  CR1_STOP_BIT  0x0200
#define  I2C_IT_BUF  0x0400
#define  I2C_IT_EVT  0x0200
#define  I2C_IT_ERR  0x0100
#define  CR1_PE_SET  0x0001
//I2c master buffer
#define i2c I2C1
/* ------------------------------------------------------------------ */

//Tx buffer pointer
static const uint8_t *tx_buf;
//Rx buffer pointer
static uint8_t *rx_buf;
//Busy semaphore
static sem_t *sem_lock;
//Read semaphore
static sem_t *sem_irq;
//Bus address
static volatile uint8_t bus_addr;
//Bus error flags
static volatile uint8_t err_flag;
//Tx counter
static volatile short tx_bytes;
//Rx counter
static volatile short rx_bytes;
//Position in the buffer
static volatile short buf_pos;

/* ------------------------------------------------------------------ */
//Private funcs
/***/
//Get last i2c event
static inline uint32_t get_last_event()
{
	return ( (uint32_t)(i2c->SR1) |(uint32_t)(i2c->SR2)<<16 )
	& 0x00FFFFFF;
}
/***/
//Send 7 bit address on the i2c bus
void send_7bit_addr(uint8_t addr)
{
	i2c->DR = addr;
}
/***/
//Send data on the bus
static inline void send_data(uint8_t data)
{
	i2c->DR = data;
}
/***/
//Read data from the bus
static inline 	uint8_t receive_data()
{
	return i2c->DR;
}
/***/
//CR1 reg enable disable
static inline 	void cr1_reg(unsigned bit, bool en)
{
	if(en) i2c->CR1 |= bit;
	else i2c->CR1 &= ~bit;
}
/***/
//ACK ON control
static inline 	void ack_on(bool on)
{
	cr1_reg( CR1_ACK_BIT, on );
}
/***/
//Generate start
static inline 	void generate_start(bool en)
{
	cr1_reg( CR1_START_BIT, en );
}
/***/
//Generate stop
static inline void generate_stop(bool en)
{
	cr1_reg( CR1_STOP_BIT, en );
}
/***/
//Clear data flags (dummy read)
static inline void clear_flags()
{
	(void)(volatile uint16_t)(i2c->SR1);
	(void)(volatile uint16_t)(i2c->DR);
}

/***/
//Control enabling disabling int in the device
static inline void devirq_on(bool en)
{
	if(en)
		/* Enable I2C interrupt */
		i2c->CR2 |=  I2C_IT_EVT| I2C_IT_ERR;
	else
		/* diasable I2C interrupt */
		 i2c->CR2 &=  ~(I2C_IT_EVT | I2C_IT_ERR);
}
/***/
int get_hwerror(void)
{
	static const int err_tbl[] =
	{
		ERR_BUS,
		ERR_ARBITRATION_LOST,
		ERR_ACK_FAILURE,
		ERR_OVERRUN,
		ERR_PEC,
		ERR_UNKNOWN,
		ERR_BUS_TIMEOUT,
	};

	for(int i=0; i<7; i++)
	{
		if(err_flag & (1<<i))
			return err_tbl[i];
	}

	return 0;
}

/* ------------------------------------------------------------------ */
//Initialize the library
errno_t i2cm_init( unsigned clk_speed)
{

	//Sema init 	i2c(_i2c), sem_lock(1,1),sem_irq(0,1),err_flag(0)
	sem_lock = isix_sem_create_limited(NULL , 1, 1 );
	if(!sem_lock) return ERR_ISIX;
	sem_irq = isix_sem_create_limited(NULL, 0, 1 );
	if(!sem_irq) return ERR_ISIX;

	//GPIO configuration
	RCC->APB2ENR |= I2C1_GPIO_ENR;
	io_config(I2C1_PORT,I2C1_SDA_PIN,GPIO_MODE_50MHZ,GPIO_CNF_ALT_OD);
	io_config(I2C1_PORT,I2C1_SCL_PIN,GPIO_MODE_50MHZ,GPIO_CNF_ALT_OD);
	io_set(I2C1_PORT,I2C1_SCL_PIN);
	io_set(I2C1_PORT,I2C1_SDA_PIN);
	//I2C module configuration
	RCC->APB1ENR |= I2C1_ENR;

	/* Enable I2C module*/
	i2c->CR1 |= CR1_PE_SET;
	/* Reset the i2c device */
	i2c->CR1 |= CR1_SWRST;
	nop();
	i2c->CR1 &= ~CR1_SWRST;

	 uint16_t tmpreg = i2c->CR2;
	 /* Clear frequency FREQ[5:0] bits */
	 tmpreg &= CR2_FREQ_RESET;
	 tmpreg |= (uint16_t)(PCLK1_HZ/1000000);
	 i2c->CR2 = tmpreg;

	 //Set speed
	 i2cm_set_speed(clk_speed);

	 /* CR1 configuration */
	 /* Get the I2Cx CR1 value */
	 tmpreg = i2c->CR1;
	 /* Clear ACK, SMBTYPE and  SMBUS bits */
	 tmpreg &= CR1_CLEAR_MASK;
	 /* Configure I2Cx: mode and acknowledgement */
	 /* Set SMBTYPE and SMBUS bits according to I2C_Mode value */
	 /* Set ACK bit according to I2C_Ack value */
	 tmpreg |= I2C_MODE_I2C | I2C_ACK_ENABLE;
	 /* Write to I2Cx CR1 */
	 i2c->CR1 = tmpreg;

	 /* Set I2Cx Own Address1 and acknowledged address */
	 i2c->OAR1 = I2C_AcknowledgedAddress_7bit;

	 i2c->SR1 = 0; i2c->SR2 = 0;

	 /* Enable interrupt controller */
	 nvic_set_priority( I2C1_EV_IRQn, IRQ_PRIO, IRQ_SUB);
	 nvic_set_priority( I2C1_ER_IRQn, IRQ_PRIO, IRQ_SUB);
	 nvic_irq_enable(I2C1_EV_IRQn,true);
	 nvic_irq_enable(I2C1_ER_IRQn,true);
	 return ERR_OK;
}

/* ------------------------------------------------------------------ */
void i2cm_set_speed(unsigned speed)
{
	 /* Disable the selected I2C peripheral to configure TRISE */
	  i2c->CR1 &= CR1_PE_RESET;
	  /* Reset tmpreg value */
	  /* Clear F/S, DUTY and CCR[11:0] bits */
	 uint16_t tmpreg = 0;
	/* Configure speed in standard mode */
	  if (speed <= 100000)
	  {
	    /* Standard mode speed calculate */
	    uint16_t result = (uint16_t)(PCLK1_HZ / (speed << 1));
	    /* Test if CCR value is under 0x4*/
	    if (result < 0x04)
	    {
	      /* Set minimum allowed value */
	      result = 0x04;
	    }
	    /* Set speed value for standard mode */
	    tmpreg |= result;
	    /* Set Maximum Rise Time for standard mode */
	    i2c->TRISE = (uint16_t)(PCLK1_HZ/1000000) + 1;
	  }
	  /* Configure speed in fast mode */
	  else /*(I2C_InitStruct->I2C_ClockSpeed <= 400000)*/
	  {

		  /* Fast mode speed calculate: Tlow/Thigh = 2 */
	     uint16_t result =(uint16_t) (PCLK1_HZ / (speed * 3));

	    /* Test if CCR value is under 0x1*/
	    if ((result & CCR_CCR_SET) == 0)
	    {
	      /* Set minimum allowed value */
	      result |= 0x0001;
	    }
	    /* Set speed value and set F/S bit for fast mode */
	    tmpreg = (uint16_t)(result | CCR_FS_SET);
	    /* Set Maximum Rise Time for fast mode */
	    i2c->TRISE = (uint16_t)(((
	    		(uint16_t)(PCLK1_HZ/1000000) *
	    		(uint16_t)(300)) /
	    		(uint16_t)(1000)) +
	    		(uint16_t)(1)
	     );
	  }
	  /* Write to I2Cx CCR */
	  i2c->CCR = tmpreg;
	  /* Enable the selected I2C peripheral */
	  i2c->CR1 |= CR1_PE_SET;

}
/* ------------------------------------------------------------------ */
int i2cm_transfer_7bit(uint8_t addr, const void* wbuffer, short wsize, void* rbuffer, short rsize)
{
	int ret;
	if( (ret=isix_sem_wait(sem_lock,ISIX_TIME_INFINITE))<0 )
	{
		return ret;
	}
	//Disable I2C irq
	devirq_on(false);
	if(wbuffer)
	{
		bus_addr = addr & ~I2C_BUS_RW_BIT;
	}
	else if(rbuffer)
	{
		bus_addr = addr | I2C_BUS_RW_BIT;
	}
	tx_buf =  (const uint8_t*)(wbuffer);
	rx_buf =  (uint8_t*)(rbuffer);
	tx_bytes = wsize;
	rx_bytes = rsize;
	buf_pos = 0;
	//ACK config
	ack_on(true);
	//Clear status flags
	clear_flags();
	//Enable I2C irq
	devirq_on(true);
	//Send the start
	generate_start(true);
	//Sem read lock
	if( (ret=isix_sem_wait(sem_irq,TRANSFER_TIMEOUT)) <0  )
	{
		if(ret==ISIX_ETIMEOUT)
		{
			devirq_on(false);
			isix_sem_signal(sem_lock);
			return ERR_TIMEOUT;
		}
		else
		{
			devirq_on(false);
			isix_sem_signal(sem_lock);
			return ret;
		}
	}
	if( (ret=get_hwerror())  )
	{
		devirq_on(false);
		err_flag = 0;
		isix_sem_signal(sem_lock);
		return ret;
	}
	devirq_on(false);
	isix_sem_signal(sem_lock);
	return ERR_OK;
}
/* ------------------------------------------------------------------ */
/* Irq handler */
void i2c1_ev_isr_vector(void) __attribute__ ((interrupt));
void i2c1_ev_isr_vector(void)
{
	uint32_t event = get_last_event();
	switch( event )
	{

	//Send address
	case I2C_EVENT_MASTER_MODE_SELECT:		//EV5
		send_7bit_addr(bus_addr);
	break;

	//Send bytes in tx mode
	case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:	//EV6
	case I2C_EVENT_MASTER_BYTE_TRANSMITTED:	//EV8
	case I2C_EVENT_MASTER_BYTE_TRANSMITTING:
	if(tx_bytes>0)
	{
		send_data(tx_buf[buf_pos++]);
		tx_bytes--;
	}
	if(tx_bytes==0)
	{
		if(rx_buf)
		{
			//Change address to read only
			bus_addr |= I2C_BUS_RW_BIT;
			ack_on(true);
			generate_start(true);
			buf_pos = 0;
		}
		else
		{
			generate_stop(true);
			devirq_on(false);
			isix_sem_signal_isr(sem_irq);
		}
	}
	break;

	//Master mode selected
	case I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED:	//EV7
	if(rx_bytes==1)
	{
		ack_on(false);
	}
	break;
	//Master byte rcv
	case I2C_EVENT_MASTER_BYTE_RECEIVED:
	if(rx_bytes>0)
	{
		rx_buf[buf_pos++] = receive_data();
		rx_bytes--;
	}
	if(rx_bytes==1)
	{
		ack_on(false);
		generate_stop(true);
	}
	else if(rx_bytes==0)
	{
		generate_stop(true);
		devirq_on(false);
		isix_sem_signal_isr(sem_irq);
	}
	break;
	//Stop generated event
	default:
		devirq_on(false);
        	generate_stop(true);
        	isix_sem_signal_isr(sem_irq);
	break;
	}
}
/* ------------------------------------------------------------------ */
/* Irq handler */
void i2c1_er_isr_vector(void) __attribute__ ((interrupt));
void i2c1_er_isr_vector(void)
{
    //SR0 only don't care about SR1
    uint16_t event = get_last_event();
    if(event & EVENT_ERROR_MASK)
    {
	err_flag = event >> 8;
	i2c->SR1 &= ~EVENT_ERROR_MASK;
	devirq_on(false);
        generate_stop(true);
	isix_sem_signal_isr(sem_irq);
    }
}
/* ------------------------------------------------------------------ */


