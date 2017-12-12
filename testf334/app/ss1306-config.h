/*
 * ssd1306_config.h
 *
 * Created: 4/29/2013 7:11:39 PM
 *  Author: kmm
 */ 


#ifndef SS1306_CONFIG_H_
#define SS1306_CONFIG_H_

#define SS1306_OLED_GEOM_W 128
#define SS1306_OLED_GEOM_H 64

#define SS1306_OLED_CLK 3U
#define SS1306_OLED_DAT 5U
#define SS1306_OLED_RST 8U
#define SS1306_OLED_DC  9U
#define SS1306_OLED_CS  6U

#define _B(x) (1U<<(x))

#define SS1306_PORT GPIOB
#define SS1306_OLED_ALL (_B(SS1306_OLED_CLK)|_B(SS1306_OLED_DAT)|_B(SS1306_OLED_RST)|_B(SS1306_OLED_DC)|_B(SS1306_OLED_CS))


#endif
