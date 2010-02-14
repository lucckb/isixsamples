/* ------------------------------------------------------------------ */
/*
 * i2c_host.cpp
 *
 *  Created on: 2010-01-20
 *      Author: lucck
 */

#include "config.hpp"
#include "i2c_host.hpp"
#include <system.h>
#include "interrupt_cntr.hpp"
#include <cstring>
#include <tiny_printf.h>
/* ------------------------------------------------------------------ */
//#define logsys tiny_printf
#define logsys(...)

/* ------------------------------------------------------------------ */
namespace dev
{
/* ------------------------------------------------------------------ */
namespace {

	//I2C PINS
	GPIO_TypeDef * const I2C1_PORT = GPIOB;
	const unsigned I2C1_SDA_PIN = 7;
	const unsigned I2C1_SCL_PIN = 6;
	const unsigned I2C1_GPIO_ENR = RCC_APB2Periph_GPIOB;
	const unsigned  I2C1_ENR     = RCC_APB1Periph_I2C1;

	const uint16_t CR1_PE_RESET = 0xFFFE;
	const uint16_t CR2_FREQ_RESET = 0xFFC0;
	const uint16_t CCR_CCR_SET = 0x0FFF;
	/* I2C F/S mask */
	const uint16_t CCR_FS_SET = 0x8000;
	const uint16_t CR1_CLEAR_MASK = 0xFBF5;
	const uint16_t CR1_SWRST = 1<<15;
	const uint16_t I2C_MODE_I2C = 0x0000;
	const uint16_t I2C_ACK_ENABLE = 0x0400;
	const uint16_t I2C_AcknowledgedAddress_7bit = 0x4000;
	const uint8_t I2C_BUS_RW_BIT = 0x01;
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
	const unsigned EVENT_ERROR_MASK = 0xff00;

}

/* ------------------------------------------------------------------ */
//Interrupt controller device
i2c_interrupt::i2c_interrupt(i2c_host &_owner):owner(_owner)
{
	if(owner.i2c==I2C1)
	{
		interrupt_cntr::register_int(interrupt_cntr::irql_i2c1,IRQ_PRIO,IRQ_SUB,this);
	}
}

/* ------------------------------------------------------------------ */
i2c_host::i2c_host(I2C_TypeDef * const _i2c, unsigned clk_speed):
		i2c(_i2c), sem_busy(1),sem_read(0), interrupt(*this)
{
	// TODO Auto-generated constructor stub
	if(_i2c==I2C1)
	{
		//GPIO configuration
		RCC->APB2ENR |= I2C1_GPIO_ENR;
		io_config(I2C1_PORT,I2C1_SDA_PIN,GPIO_MODE_50MHZ,GPIO_CNF_ALT_OD);
		io_config(I2C1_PORT,I2C1_SCL_PIN,GPIO_MODE_50MHZ,GPIO_CNF_ALT_OD);
		io_set(I2C1_PORT,I2C1_SCL_PIN);
		io_set(I2C1_PORT,I2C1_SDA_PIN);
		//I2C module configuration
		RCC->APB1ENR |= I2C1_ENR;
	}
	/* Enable I2C module*/
	i2c->CR1 |= CR1_PE_SET;
	/* Reset the i2c device */
	i2c->CR1 |= CR1_SWRST;
	nop();
	i2c->CR1 &= ~CR1_SWRST;

	 uint16_t tmpreg = i2c->CR2;
	 /* Clear frequency FREQ[5:0] bits */
	 tmpreg &= CR2_FREQ_RESET;
	 tmpreg |= static_cast<uint16_t>(config::PCLK1_HZ/1000000);
	 i2c->CR2 = tmpreg;

	 //Set speed
	 set_speed(clk_speed);

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
	 /* Enable I2C interrupt */
	 devirq_on();

	 /* Enable interrupt controller */
	 interrupt_cntr::enable_int(interrupt_cntr::irql_i2c1);
}

/* ------------------------------------------------------------------ */
void i2c_host::set_speed(unsigned speed)
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
	    uint16_t result = static_cast<uint16_t>(config::PCLK1_HZ / (speed << 1));
	    /* Test if CCR value is under 0x4*/
	    if (result < 0x04)
	    {
	      /* Set minimum allowed value */
	      result = 0x04;
	    }
	    /* Set speed value for standard mode */
	    tmpreg |= result;
	    /* Set Maximum Rise Time for standard mode */
	    i2c->TRISE = static_cast<uint16_t>(config::PCLK1_HZ/1000000) + 1;
	  }
	  /* Configure speed in fast mode */
	  else /*(I2C_InitStruct->I2C_ClockSpeed <= 400000)*/
	  {

		  /* Fast mode speed calculate: Tlow/Thigh = 2 */
	     uint16_t result = static_cast<uint16_t> (config::PCLK1_HZ / (speed * 3));

	    /* Test if CCR value is under 0x1*/
	    if ((result & CCR_CCR_SET) == 0)
	    {
	      /* Set minimum allowed value */
	      result |= 0x0001;
	    }
	    /* Set speed value and set F/S bit for fast mode */
	    tmpreg = static_cast<uint16_t>(result | CCR_FS_SET);
	    /* Set Maximum Rise Time for fast mode */
	    i2c->TRISE = static_cast<uint16_t>(((
	    	    static_cast<uint16_t>(config::PCLK1_HZ/1000000) *
	    		static_cast<uint16_t>(300)) /
	    		static_cast<uint16_t>(1000)) +
	    		static_cast<uint16_t>(1)
	     );
	  }
	  /* Write to I2Cx CCR */
	  i2c->CCR = tmpreg;
	  /* Enable the selected I2C peripheral */
	  i2c->CR1 |= CR1_PE_SET;

}

/* ------------------------------------------------------------------ */
int i2c_host::i2c_transfer_7bit(uint8_t addr, const void* wbuffer, short wsize, void* rbuffer, short rsize)
{
	int ret;
	if( (ret=sem_busy.wait(TRANSFER_TIMEOUT))<0 )
	{
		if(ret==isix::ISIX_ETIMEOUT)
		{
			sem_busy.signal();
			return get_hwerror();
		}
		else
		{
			return ret;
		}
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
	tx_buf =  static_cast<const uint8_t*>(wbuffer);
	rx_buf =  static_cast<uint8_t*>(rbuffer);
	tx_bytes = wsize;
	rx_bytes = rsize;
	buf_pos = 0;
	err_flag = 0;
	//ACK config
	ack_on(true);
	//Enable I2C irq
	devirq_on();
	//Send the start
	generate_start();
	//Sem read lock
	if(rbuffer)
	{
		if( (ret=sem_read.wait(TRANSFER_TIMEOUT))<0 )
		{
			if(ret==isix::ISIX_ETIMEOUT)
			{
				sem_read.signal();
				return get_hwerror();
			}
			else
			{
				return ret;
			}
		}
	}
	return ERR_OK;
}


/* ------------------------------------------------------------------ */
//I2c interrupt handler
void i2c_interrupt::isr()
{
	uint32_t event = owner.get_last_event();
	switch( event )
	{

	//Send address
	case I2C_EVENT_MASTER_MODE_SELECT:		//EV5
		logsys("AS>>%02x\r\n",owner.bus_addr);
		owner.send_7bit_addr(owner.bus_addr);
	break;

	//Send bytes in tx mode
	case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:	//EV6
	case I2C_EVENT_MASTER_BYTE_TRANSMITTED:	//EV8
		logsys("[%d]\r\n",owner.tx_bytes);
		if(owner.tx_bytes>0)
		{
			owner.send_data(owner.tx_buf[owner.buf_pos++]);
			owner.tx_bytes--;
		}
		if(owner.tx_bytes==0)
		{
			if(owner.rx_buf)
			{
				logsys("STA>*\r\n");
				//Change address to read only
				owner.bus_addr |= I2C_BUS_RW_BIT;
				owner.ack_on(true);
				owner.generate_start();
				owner.buf_pos = 0;
			}
			else
			{
				logsys(">STO\r\n");
				owner.generate_stop();
				if(owner.sem_busy.getval()==0)
				{
					//tiny_printf("+");
					owner.sem_busy.signal_isr();
				}
			}

		}
	break;

	//Master mode selected
	case I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED:	//EV7
		if(owner.rx_bytes==1)
		{
			owner.ack_on(false);
			//owner.generate_stop();
			//tiny_printf(",");
		}
	break;
	//Master byte rcv
	case I2C_EVENT_MASTER_BYTE_RECEIVED:
		logsys("<%d>\r\n",owner.rx_bytes);
		if(owner.rx_bytes>0)
		{
			owner.rx_buf[owner.buf_pos++] = owner.receive_data();
			owner.rx_bytes--;
		}
		if(owner.rx_bytes==1)
		{
			owner.ack_on(false);
			owner.generate_stop();
	    }
		else if(owner.rx_bytes==0)
		{
			owner.generate_stop();
			if(owner.sem_read.getval()==0)
			{
				owner.sem_read.signal_isr();
			}
			if(owner.sem_busy.getval()==0)
			{
				owner.sem_busy.signal_isr();
			}
		}
	break;

	//Stop generated event
	default:
		tiny_printf("!!!0x%08x!!!\r\n",event);
		if(event & EVENT_ERROR_MASK)
		{
			owner.err_flag = event >> 8;
			owner.i2c->SR1 &= ~EVENT_ERROR_MASK;
		}
		else
		{
			owner.clear_flags();
		}
	break;
	}
}
/* ------------------------------------------------------------------ */
//Translate hardware error to the error code
int i2c_host::get_hwerror()
{
	static const int err_tbl[] =
	{
		ERR_BUS,
		ERR_ARBITRATION_LOST,
		ERR_ACK_FAILURE,
		ERR_OVERRUN,
		ERR_PEC,
		ERR_UNKNOWN,
		ERR_BUS_TIMEOUT
	};
	for(int i=0; i<7; i++)
	{
		if(err_flag & (1<<i))
			return err_tbl[i];
	}
	return ERR_UNKNOWN;
}
/* ------------------------------------------------------------------ */

}
/* ------------------------------------------------------------------ */
