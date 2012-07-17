/* ------------------------------------------------------------------ */
/*
 * nokia_display.cpp
 * Nokia N3310 display class
 *  Created on: 2010-01-03
 *      Author: lucck
 */
/* The display PIN assignment LCD N3310 (84x48)
 * SCLK - PA5 (SCLK) (typ Alternate Output 2)
 * SDIN - PA7 (MOSI) (typ Alternate Output 2)
 * DC   - PD0 (Output)
 * SEL  - PA4 (Output)
 * RES -  PD1  (Output)
 * Device connected to the hardware pin SPI1
 */
/* ------------------------------------------------------------------ */
#include "nokia_display.hpp"
#include "img_def.hpp"
#include <stm32lib.h>
#include <isix.h>
/* ------------------------------------------------------------------ */
//Application namespace
namespace app
{
/* ------------------------------------------------------------------ */
//Unnamed namespace
namespace
{
	//SPI port
	GPIO_TypeDef* const SPI_PORT = GPIOA;
	//SPI pin bits
	const unsigned SSEL_BIT = 4;
	const unsigned MOSI_BIT = 7;
	const unsigned SCK_BIT = 5;
	//Display control bits
	GPIO_TypeDef* const CTRL_PORT = GPIOD;
	const unsigned DC_BIT = 0;
	const unsigned RES_BIT = 1;
	//Hardware defs
	const unsigned CR1_SPE_Set = 0x0040;
	const unsigned SPI_Mode_Select = 0xF7FF;
	const unsigned INIT_DELAY = 1000000;
	const unsigned SSEL_DELAY = 64;
	const unsigned BYTES_ON_DISPLAY = 504;
	//Characters table
	const char cg_rom[]=
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

}
/* ------------------------------------------------------------------ */
//Display constructor
nokia_display::nokia_display()
{
	//Initialize hardware
	hw_init();
	//Initialize LCD
	lcd_init();
}
/* ------------------------------------------------------------------ */
//Set DC pin in state x
inline void nokia_display::dc_pin(bool en)
{
    if( en) stm32::gpio_set(CTRL_PORT,DC_BIT);
	else    stm32::gpio_clr(CTRL_PORT,DC_BIT);
}

/* ------------------------------------------------------------------ */
//Set RES pin in state x
inline void nokia_display::res_pin(bool en)
{
    if(en) stm32::gpio_set(CTRL_PORT,RES_BIT);
	else   stm32::gpio_clr(CTRL_PORT,RES_BIT);
}
/* ------------------------------------------------------------------ */
//Set SEL pin in state x
inline void nokia_display::sel_pin(bool en)
{
	busy_delay(SSEL_DELAY);
    if(en) stm32::gpio_set(SPI_PORT,SSEL_BIT);
	else   stm32::gpio_clr(SPI_PORT,SSEL_BIT);
}

/* ------------------------------------------------------------------ */
//Hardware initialization method
void nokia_display::hw_init()
{
	using namespace stm32;
	RCC->APB2ENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOD | RCC_APB2Periph_SPI1;

	//SPI MISO (Input)
	//gpio_config(SPI_PORT,MISO_BIT,GPIO_MODE_50MHZ,GPIO_CNF_ALT_PP);
	//SPI MOSI (Output)
	gpio_config(SPI_PORT,MOSI_BIT,GPIO_MODE_50MHZ,GPIO_CNF_ALT_PP);
	//SPI SCK (Output)
	gpio_config(SPI_PORT,SCK_BIT,GPIO_MODE_50MHZ,GPIO_CNF_ALT_PP);
	//SSEL
	gpio_config(SPI_PORT,SSEL_BIT,GPIO_MODE_50MHZ,GPIO_CNF_GPIO_PP);

    //Control lines directions
	gpio_config(CTRL_PORT,DC_BIT,GPIO_MODE_50MHZ,GPIO_CNF_GPIO_PP);
	gpio_config(CTRL_PORT,RES_BIT,GPIO_MODE_50MHZ,GPIO_CNF_GPIO_PP);

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
void nokia_display::hw_spi_write(unsigned char byte)
{
	 while(!(SPI1->SR & SPI_I2S_FLAG_TXE));
	 SPI1->DR = byte;
	 while(SPI1->SR & SPI_I2S_FLAG_BSY); // Wait until sending is over
}
/* ------------------------------------------------------------------ */
//Initialize LCD display
void nokia_display::lcd_init()
{

    //Display reset
    res_pin(0);
    isix::isix_wait_ms(250);
    res_pin(1);

    //Initial state
    sel_pin(0);                 // Activate
    dc_pin(0);                  // Instructions

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

    dc_pin(1);                  //Data mode
    sel_pin(1);                 //Display CS off.
    sel_pin(0);                 //Display CS on

    //Clear RAM display
    for(unsigned i=0;i<BYTES_ON_DISPLAY;i++)
	    hw_spi_write(0x0);
	//Deactivate select signal
    sel_pin(1);
}
/* ------------------------------------------------------------------ */
//Very short delay busy wait method
void nokia_display::busy_delay(unsigned delay)
{
	for(unsigned i=0;i<delay;i++)
		asm volatile("nop");
}
/* ------------------------------------------------------------------ */
//Write character at current position
void nokia_display::put_char(char code)
{
    std::size_t code_point;
	//Data
    dc_pin(1);
	//Char in array calculation
    code_point = static_cast<std::size_t>(code) * 6;
	//Write character.
	sel_pin(0);
    hw_spi_write(cg_rom[code_point++]);
	hw_spi_write(cg_rom[code_point++]);
	hw_spi_write(cg_rom[code_point++]);
	hw_spi_write(cg_rom[code_point++]);
	hw_spi_write(cg_rom[code_point++]);
	hw_spi_write(cg_rom[code_point]);
    //Write end
    sel_pin(1);
}
/* ------------------------------------------------------------------ */
//Set cursor at selected pos (14 cols, 6 rows)
void nokia_display::set_position(unsigned x, unsigned y)
{
	x=x*6;
	//Write activation
	sel_pin(0);
    //Instr
    dc_pin(0);
	hw_spi_write(y|0x40);       //set row position
	hw_spi_write(x|0x80);       //set col position
	// Write stop
    sel_pin(1);
    dc_pin(1);
}
/* ------------------------------------------------------------------ */
//Display string at selected position
void nokia_display::put_string(const char *str, unsigned  x, unsigned y)
{
	//Set display pos.
    set_position(x,y);
	dc_pin(1);
    while(*str) put_char(*str++);
}
/* ------------------------------------------------------------------ */
//Put bitmap on the screen
void nokia_display::put_bitmap(const images::img_def &image)
{
	dc_pin(0);
	sel_pin(0);
    hw_spi_write(0x40);     //Zero col. counter
    hw_spi_write(0x80);     //Zero row counter
    hw_spi_write(0x22);     //Horizontal mode
    sel_pin(1);
    dc_pin(1);                  //Switch to data

    int width = (image.width>84)?(84):(image.width);
    int height = (image.height>48)?(48):(image.height);

    dc_pin(1);      //Data mode
    sel_pin(0);     //Activation
    //Width multiplier
    int width_mult = image.width/8 + !!(image.width%8);
    for(int w=0; w<width; w++)
    {
    	int h;
    	uint8_t sbyte=0;
    	for(h=0; h<height; h++)
    	{
    		if( image.data[h*width_mult + w/8] & (1<<(7-w%8)) )
    			sbyte |= 1<<(h%8);

    		if(h%8==7)
    		{
    			hw_spi_write(sbyte);
    			sbyte = 0;
    		}
    	}
    	if(h%8!=0)
    	{
    		hw_spi_write(sbyte);
    	}
    	//Fill the rest of the display
    	for(int col=h/8+!!(h%8); col<6; col++)
    		hw_spi_write(0x00);
    }
    sel_pin(1);
    //Initial mode
    sel_pin(0);
    dc_pin(0);              //Instructions
    hw_spi_write(0x40);     //Reset row counter
    hw_spi_write(0x80);     //Reset col counter
    hw_spi_write(0x20);     //Horizontal mode
    sel_pin(1);
    dc_pin(1);              //Data mode
}
/* ------------------------------------------------------------------ */
//Clear the display
void nokia_display::clear()
{
	set_position(0,0);
	dc_pin(1);                  //Data mode
	sel_pin(1);                 //Activate CS
	sel_pin(0);                 //Deactivate CS

	//Zero display memory.
	for(unsigned i=0;i<BYTES_ON_DISPLAY;i++)
		hw_spi_write(0x0);
	//Deactivate select signal
	sel_pin(1);
}
/* ------------------------------------------------------------------ */
}
/* ------------------------------------------------------------------ */
