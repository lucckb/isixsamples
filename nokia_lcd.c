/*
 * nokia_lcd.c
 *
 *  Created on: 18-09-2010
 *      Author: lucck
 */

/* The display PIN assignment LCD N3310 (84x48)
 * SCLK - PA5 (SCLK) (typ Alternate Output 2)
 * SDIN - PA7 (MOSI) (typ Alternate Output 2)
 * DC   - PD0 (Output)
 * SEL  - PA4 (Output)
 * RES -  PD1  (Output)
 * Device connected to the hardware pin SPI1 */
/* ------------------------------------------------------------------ */
#include "nokia_lcd.h"
#include <stm32f10x_lib.h>
#include <isix.h>
/* ------------------------------------------------------------------ */
//! Define if new uncompatible display will be used
#define NEW_3310_INCOMPATIBILE_DISP

/* ------------------------------------------------------------------ */
//SPI port
#define SPI_PORT GPIOA
//SPI pin bits
#define SSEL_BIT 4
#define MOSI_BIT 7
#define SCK_BIT 5
	//Display control bits
#define CTRL_PORT GPIOD
#define DC_BIT 0
#define RES_BIT 1
	//Hardware defs
#define CR1_SPE_Set 0x0040
#define SPI_Mode_Select  0xF7FF
#define NIT_DELAY  1000000
#define SSEL_DELAY  64
#define BYTES_ON_DISPLAY  504


//Characters table
static const char cg_rom[]=
{
	    0x48,0x54,0x56,0x54,0x20,0,     //kod 0
	    0x20,0x54,0x54,0xd4,0x78,0,     //kod 1
	    0x38,0x44,0x46,0x44,0x20,0,     //kod 2
	    0,0,0,0,0,0,                    //kod 3
	    0,0,0,0,0,0,                    //kod 4
	    0,0,0,0,0,0,                    //kod 5
	    0,0,0,0,0,0,                    //kod 6
	    0,0,0,0,0,0,                    //kod 7
	    0,0,0,0,0,0,                    //kod 8
	    0,0,0,0,0,0,                    //kod 9
	    0,0,0,0,0,0,                    //kod A
	    0,0,0,0,0,0,                    //kod B
	    0,0,0,0,0,0,                    //kod 12
	    0,0,0,0,0,0,                    //kod 13
	    0,0,0,0,0,0,                    //kod 14
	    0,0,0,0,0,0,                    //kod 15
	    0,0,0,0,0,0,                    //kod 16
	    0,0,0,0,0,0,                    //kod 17
	    0,0,0,0,0,0,                    //kod 18
	    0,0,0,0,0,0,                    //kod 19
	    0,0,0,0,0,0,                    //kod 20
	    0,6,9,9,6,0,                    //kod 21 stopnie C
	    0x2a,0x1c,0x3e,0x1c,0x2a,0,     //kod 22 (sloneczko)
	    0x24,0x46,0x46,0x3e,0x06,0x04,  //kod 23(parasolka)
	    0,0xc0,0xe0,0xf0,0xc8,0x88,     //kod 24 (szpula magnet)
	    0x88,0xc8,0xf0,0xe0,0xc0,0,     //kod 25
	    0,0xc0,0x20,0x10,0x38,0xf8,     //kod 26
	    0xf8,0x38,0x10,0x20,0xc0,0,     //kod 27
	    0,3,4,8,0x1c,0x1f,              //kod 28
	    0x1f,0x1c,0x08,4,3,0,           //kod 29
	    0,3,7,0x0f,0x13,0x11,           //kod 30
	    0x11,0x13,0x0f,7,3,0,           //kod 31
	    0,0,0,0,0,0,                    //kod spacja
	    0,0,0xf2,0,0,0,                 //kod !
	    0,3,0,3,0,0,                    //kod "
	    0x14,0x7f,0x14,0x7f,0x14,0,     //kod #
	    0x24,0x2a,0x7f,0x2a,0x12,0,     //kod $
	    0x23,0x13,8,0x64,0x62,0,        //kod %
	    0x36,0x49,0x55,0x22,0x50,0,     //kod &
	    0,5,3,0,0,0,                    //kod '
	    0,0x1c,0x22,0x41,0,0,           //kod (
	    0,0x41,0x22,0x1c,0,0,           //kod )
	    0x14,8,0x3e,8,0x14,0,           //kod *
	    8,8,0x3e,8,8,0,                 //kod +
	    0,0x50,0x30,0,0,0,              //kod ,
	    8,8,8,8,8,8,                    //kod -
	    0,0x60,0x60,0,0,0,              //kod .
	    0x20,0x10,8,4,2,0,              //kod /
	    0x3e,0x51,0x49,0x45,0x3e,0,     //kod 0
	    0,0x42,0x7f,0x40,0,0,           //kod 1
	    0x42,0x61,0x51,0x49,0x46,0,     //kod 2
	    0x21,0x41,0x45,0x4b,0x31,0,     //kod 3
	    0x18,0x14,0x12,0x7f,0x10,0,     //kod 4
	    0x27,0x45,0x45,0x45,0x39,0,     //kod 5
	    0x3c,0x4a,0x49,0x49,0x30,0,     //kod 6
	    1,0x71,9,5,3,0,                 //kod 7
	    0x36,0x49,0x49,0x49,0x36,0,     //kod 8
	    6,0x49,0x49,0x29,0x1e,0,        //kod 9
	    0,0x36,0x36,0,0,0,              //kod :
	    0,0x56,0x36,0,0,0,              //kod ;
	    8,0x14,0x22,0x41,0,0,           //kod <
	    0x14,0x14,0x14,0x14,0x14,0x00,  //kod=
	    0,0x41,0x22,0x14,8,0,           //kod >
	    2,1,0x51,9,6,0,                 //kod ?
	    0x30,0x49,0x79,0x41,0x3e,0,     //kod @
	    0x7E,0x11,0x11,0x11,0x7E,0x00,
	    0x7F,0x49,0x49,0x49,0x36,0x00,
	    0x3E,0x41,0x41,0x41,0x22,0x00,
	    0x7F,0x41,0x41,0x22,0x1C,0x00,
	    0x7F,0x49,0x49,0x49,0x41,0x00,
	    0x7F,0x09,0x09,0x09,0x01,0x00,
	    0x3E,0x41,0x49,0x49,0x7A,0x00,
	    0x7F,0x08,0x08,0x08,0x7F,0x00,
	    0x00,0x41,0x7F,0x41,0x00,0x00,
	    0x20,0x40,0x41,0x3F,0x01,0x00,
	    0x7F,0x08,0x14,0x22,0x41,0x00,
	    0x7F,0x40,0x40,0x40,0x40,0x00,
	    0x7F,0x02,0x0C,0x02,0x7F,0x00,
	    0x7F,0x04,0x08,0x10,0x7F,0x00,
	    0x3E,0x41,0x41,0x41,0x3E,0x00,
	    0x7F,0x09,0x09,0x09,0x06,0x00,
	    0x3E,0x41,0x51,0x21,0x5E,0x00,
	    0x7F,0x09,0x19,0x29,0x46,0x00,
	    0x46,0x49,0x49,0x49,0x31,0x00,
	    0x01,0x01,0x7F,0x01,0x01,0x00,
	    0x3F,0x40,0x40,0x40,0x3F,0x00,
	    0x1F,0x20,0x40,0x20,0x1F,0x00,
	    0x3F,0x40,0x38,0x40,0x3F,0x00,
	    0x63,0x14,0x08,0x14,0x63,0x00,
	    0x07,0x08,0x70,0x08,0x07,0x00,
	    0x61,0x51,0x49,0x45,0x43,0x00,
	    0x00,0x7F,0x41,0x41,0x00,0x00,
	    0x02,0x04,0x08,0x10,0x20,0x00,
	    0x00,0x41,0x41,0x7F,0x00,0x00,
	    0x04,0x02,0x01,0x02,0x04,0x00,
	    0x40,0x40,0x40,0x40,0x40,0x00,
	    0x00,0x01,0x02,0x04,0x00,0x00,
	    0x20,0x54,0x54,0x54,0x78,0x00,
	    0x7F,0x48,0x44,0x44,0x38,0x00,
	    0x38,0x44,0x44,0x44,0x20,0x00,
	    0x38,0x44,0x44,0x48,0x7F,0x00,
	    0x38,0x54,0x54,0x54,0x18,0x00,
	    0x08,0x7E,0x09,0x01,0x02,0x00,
	    0x0C,0x52,0x52,0x52,0x3E,0x00,
	    0x7F,0x08,0x04,0x04,0x78,0x00,
	    0x00,0x44,0x7D,0x40,0x00,0x00,
	    0x20,0x40,0x44,0x3D,0x00,0x00,
	    0x7F,0x10,0x28,0x44,0x00,0x00,
	    0x00,0x41,0x7F,0x40,0x00,0x00,
	    0x7C,0x04,0x18,0x04,0x78,0x00,
	    0x7C,0x08,0x04,0x04,0x78,0x00,
	    0x38,0x44,0x44,0x44,0x38,0x00,
	    0x7C,0x14,0x14,0x14,0x08,0x00,
	    0x08,0x14,0x14,0x18,0x7C,0x00,
	    0x7C,0x08,0x04,0x04,0x08,0x00,
	    0x48,0x54,0x54,0x54,0x20,0x00,
	    0x04,0x3F,0x44,0x40,0x20,0x00,
	    0x3C,0x40,0x40,0x20,0x7C,0x00,
	    0x1C,0x20,0x40,0x20,0x1C,0x00,
	    0x3C,0x40,0x30,0x40,0x3C,0x00,
	    0x44,0x28,0x10,0x28,0x44,0x00,
	    0x0C,0x50,0x50,0x50,0x3C,0x00,
	    0x44,0x64,0x54,0x4C,0x44,0x00,
	    0x00,0x08,0x36,0x41,0x00,0x00,
	    0x00,0x00,0x7F,0x00,0x00,0x00,
	    0x00,0x41,0x36,0x08,0x00,0x00,
	    0x10,0x08,0x08,0x10,0x08,0x00,
	    0x78,0x46,0x41,0x46,0x78,0x00,
};
/* ------------------------------------------------------------------ */
#ifdef NEW_3310_INCOMPATIBILE_DISP
unsigned char cx,cy;
#endif

/* ------------------------------------------------------------------ */
//Set DC pin in state x
static inline void dc_pin(bool en)
{
	if(en) io_set(CTRL_PORT,DC_BIT);
	else   io_clr(CTRL_PORT,DC_BIT);
}

/* ------------------------------------------------------------------ */
//Set RES pin in state x
static inline void res_pin(bool en)
{
	if(en) io_set(CTRL_PORT,RES_BIT);
	else   io_clr(CTRL_PORT,RES_BIT);
}
/* ------------------------------------------------------------------ */
//Very short delay busy wait method
static void busy_delay(unsigned delay)
{
	for(unsigned i=0;i<delay;i++)
		asm volatile("nop");
}

/* ------------------------------------------------------------------ */
//Set SEL pin in state x
static inline void sel_pin(bool en)
{
	busy_delay(SSEL_DELAY);
	if(en) io_set(SPI_PORT,SSEL_BIT);
	else   io_clr(SPI_PORT,SSEL_BIT);
}

/* ------------------------------------------------------------------ */
//Hardware initialization method
static void hw_init(void)
{
	RCC->APB2ENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOD | RCC_APB2Periph_SPI1;

	//SPI MISO (Input)
	//io_config(SPI_PORT,MISO_BIT,GPIO_MODE_50MHZ,GPIO_CNF_ALT_PP);
	//SPI MOSI (Output)
	io_config(SPI_PORT,MOSI_BIT,GPIO_MODE_50MHZ,GPIO_CNF_ALT_PP);
	//SPI SCK (Output)
	io_config(SPI_PORT,SCK_BIT,GPIO_MODE_50MHZ,GPIO_CNF_ALT_PP);
	//SSEL
	io_config(SPI_PORT,SSEL_BIT,GPIO_MODE_50MHZ,GPIO_CNF_GPIO_PP);

    //Control lines directions
	io_config(CTRL_PORT,DC_BIT,GPIO_MODE_50MHZ,GPIO_CNF_GPIO_PP);
	io_config(CTRL_PORT,RES_BIT,GPIO_MODE_50MHZ,GPIO_CNF_GPIO_PP);

    //Linia DC stan 1
    dc_pin(1);
    //Linia SEL stan 1
	sel_pin(1);
    //Linia RES stan 1
    res_pin(1);
    SPI1->CR1 = SPI_Direction_1Line_Tx | SPI_Mode_Master |
            SPI_DataSize_8b | SPI_CPOL_Low | SPI_CPHA_1Edge |SPI_BaudRatePrescaler_16|
            SPI_FirstBit_MSB |SPI_NSS_Soft;
    // Enable SPI1
    SPI1->CR1 |= CR1_SPE_Set;

}
/* ------------------------------------------------------------------ */
//Write to the SPI
static void hw_spi_write(unsigned char byte)
{
	 while(!(SPI1->SR & SPI_I2S_FLAG_TXE));
	 SPI1->DR = byte;
	 while(SPI1->SR & SPI_I2S_FLAG_BSY); // Wait until sending is over
}
/* ------------------------------------------------------------------ */

#ifdef NEW_3310_INCOMPATIBILE_DISP
static void nlcd_data(unsigned char data)
{
	hw_spi_write(data);
	cx++;
	if (cx > 83)
	{

		sel_pin(1);
		nop();
		nlcd_set_position(0,cy+1);
		nop();
		sel_pin(0);
	}
	if (cy > 5)
	{
		sel_pin(1);
		nop();
		nlcd_set_position(cx,0);
		nop();
		sel_pin(0);
	}
}
#else
static inline void nlcd_data(unsigned char data)
{
	hw_spi_write(data);
}
#endif
/* ------------------------------------------------------------------ */
//Clear the display
void nlcd_clear(void)
{
	nlcd_set_position(0,0);
	dc_pin(1);                  //Data mode
	sel_pin(1);                 //Activate CS
	sel_pin(0);                 //Deactivate CS

	//Zero display memory.
	for(unsigned i=0;i<BYTES_ON_DISPLAY;i++)
		nlcd_data(0x00);
	//Deactivate select signal
	sel_pin(1);
}
/* ------------------------------------------------------------------ */
//Initialize LCD display
void nlcd_init(void)
{
	//Initialize hardware
	hw_init();
    //Display reset
	isix_wait_ms(250);
	res_pin(0);
    isix_wait_ms(250);
    res_pin(1);

    //Initial state
    sel_pin(0);                 // Activate
    dc_pin(0);                  // Instructions

#ifdef NEW_3310_INCOMPATIBILE_DISP
    hw_spi_write(0x21);
    hw_spi_write(0xb4);
    hw_spi_write(0x45);
    hw_spi_write(0x06);
    hw_spi_write(0x14);
    hw_spi_write(0x20);
    hw_spi_write(0x0c);
    hw_spi_write(0x40);
    hw_spi_write(0x80);
#else
    hw_spi_write(0x21);			//Display initialization
    hw_spi_write(0x05);
    hw_spi_write(0xc8);
    hw_spi_write(0x06);
    hw_spi_write(0x14);
    hw_spi_write(0x20);
    hw_spi_write(0x01);
    hw_spi_write(0x0c);
    hw_spi_write(0x40);
    hw_spi_write(0x80);
#endif

    dc_pin(1);                  //Data mode
    sel_pin(1);                 //Display CS off.
    //Clear the display
    nlcd_clear();
}


/* ------------------------------------------------------------------ */
//Write character at current position
void nlcd_put_char(char code)
{
    size_t code_point;
	//Data
    dc_pin(1);
	//Char in array calculation
    code_point = (size_t)code * 6;
	//Write character.
	sel_pin(0);
    nlcd_data(cg_rom[code_point++]);
    nlcd_data(cg_rom[code_point++]);
    nlcd_data(cg_rom[code_point++]);
    nlcd_data(cg_rom[code_point++]);
    nlcd_data(cg_rom[code_point++]);
    nlcd_data(cg_rom[code_point]);
    //Write end
    sel_pin(1);
}
/* ------------------------------------------------------------------ */
//Set cursor at selected pos (14 cols, 6 rows)
void nlcd_set_position(unsigned x, unsigned y)
{
	x=x*6;
	//Write activation
	sel_pin(0);
    //Instr
    dc_pin(0);
#ifdef NEW_3310_INCOMPATIBILE_DISP
    hw_spi_write(((y+1)&0x07)|0x40);
    cx = x; cy = y;
#else
	hw_spi_write((y&0x07)|0x40);       //set row position
#endif
	hw_spi_write((x&0x7f)|0x80);       //set col position
	// Write stop
    sel_pin(1);
    dc_pin(1);
}
/* ------------------------------------------------------------------ */
//Display string at selected position
void nlcd_put_string(const char *str, unsigned  x, unsigned y)
{
	//Set display pos.
    nlcd_set_position(x,y);
	dc_pin(1);
    while(*str) nlcd_put_char(*str++);
}
/* ------------------------------------------------------------------ */
