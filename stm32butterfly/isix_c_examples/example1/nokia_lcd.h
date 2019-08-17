 
/*
 * nokia_lcd.h
 *
 *  Created on: 18-09-2010
 *      Author: lucck
 */
 
#ifndef NOKIA_LCD_H_
#define NOKIA_LCD_H_

 
//! Initialize LCD display
void nlcd_init(void);

 

/** Write character at current position
 * @param[in] code - ASCII char code
 */
void nlcd_put_char(char code);

 
/**
 * Set display pointer at selected position
 * @param[in] x X-coord position
 * @param[in] y Y-coord position
 */
void nlcd_set_position(unsigned x, unsigned y);

 
/**
 * Display string at selected position
 * @param[in] str String to write on display
 * @param[in] x X-pos of the string
 * @param[in] y Y-pos of the string
 */
void nlcd_put_string(const char *str, unsigned  x, unsigned y);

 

//! Clear the whole screen
void nlcd_clear(void);

 
#endif /* NOKIA_LCD_H_ */
