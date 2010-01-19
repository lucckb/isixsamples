/* ------------------------------------------------------------------ */
/*
 * nokia_display.hpp
 * Nokia N3310 display class header
 *  Created on: 2010-01-03
 *      Author: lucck
 */
/* ------------------------------------------------------------------ */
#ifndef NOKIA_DISPLAY_HPP_
#define NOKIA_DISPLAY_HPP_
/* ------------------------------------------------------------------ */
//Forward declaration
namespace images
{
	struct img_def;
}

/* ------------------------------------------------------------------ */
//Application namespace
namespace app
{
/* ------------------------------------------------------------------ */
//Nokia display class
class nokia_display
{
public:
	//Constructor
	nokia_display();
	//Put char
	void put_char(char code);
	//Set position
	void set_position(unsigned x, unsigned y);
	//Put string
	void put_string(const char *str, unsigned  x, unsigned y);
	//Put bitmap
	void put_bitmap(const images::img_def &image);
	//Clear display
	void clear();
private:
	inline void dc_pin(bool en);
	inline void res_pin(bool en);
	inline void sel_pin(bool en);
	void hw_init();
	void hw_spi_write(unsigned char byte);
	void lcd_init();
	void busy_delay(unsigned delay);
};

/* ------------------------------------------------------------------ */
}
/* ------------------------------------------------------------------ */
#endif /* NOKIA_DISPLAY_HPP_ */
