//
//  Font data for Courier New 12pt
//

#include <periph/drivers/display/mono/lcd_font.hpp>
#include "resources.hpp"

namespace app {
namespace res {


// Character bitmaps for Courier New 12pt
static constexpr unsigned char courierNew_12ptBitmaps[] =
{
	// @0 '!' (3 pixels wide)
	//
	//  #
	//  #
	//  #
	//  #
	//  #
	//  #
	//  #
	//
	// ###
	// ###
	//
	//
	//
	//
	0x00, 0xFE, 0x00,
	0x06, 0x06, 0x06,

	// @6 '"' (5 pixels wide)
	//
	// ## ##
	// ## ##
	// #  #
	// #  #
	//
	//
	//
	//
	//
	//
	//
	//
	//
	//
	0x1E, 0x06, 0x00, 0x1E, 0x06,
	0x00, 0x00, 0x00, 0x00, 0x00,

	// @16 '#' (8 pixels wide)
	//    #  #
	//    #  #
	//   #  #
	//   #  #
	// ########
	//   #  #
	//   #  #
	// ########
	//   #  #
	//   #  #
	//  #  #
	//  #  #
	//
	//
	//
	0x90, 0x90, 0xFC, 0x93, 0x90, 0xFC, 0x93, 0x90,
	0x00, 0x0C, 0x03, 0x00, 0x0C, 0x03, 0x00, 0x00,

	// @32 '$' (6 pixels wide)
	//   #
	//  #####
	// #    #
	// #
	// #
	//  ####
	//      #
	//      #
	// #    #
	// #####
	//   #
	//   #
	//
	//
	//
	0x1C, 0x22, 0x23, 0x22, 0x22, 0xC6,
	0x03, 0x02, 0x0E, 0x02, 0x02, 0x01,

	// @44 '%' (6 pixels wide)
	//
	//  ##
	// #  #
	// #  #
	//  ##
	//    ###
	// ###
	//    ##
	//   #  #
	//   #  #
	//    ##
	//
	//
	//
	//
	0x4C, 0x52, 0x52, 0xAC, 0xA0, 0x20,
	0x00, 0x00, 0x03, 0x04, 0x04, 0x03,

	// @56 '&' (6 pixels wide)
	//
	//
	//
	//   ###
	//  #
	//  #
	//  ##
	// # #  #
	// #  ##
	// #   #
	//  #####
	//
	//
	//
	//
	0x80, 0x70, 0xC8, 0x08, 0x08, 0x80,
	0x03, 0x04, 0x04, 0x05, 0x07, 0x04,

	// @68 ''' (3 pixels wide)
	//
	// ###
	// ###
	//  #
	//  #
	//  #
	//
	//
	//
	//
	//
	//
	//
	//
	//
	0x06, 0x3E, 0x06,
	0x00, 0x00, 0x00,

	// @74 '(' (3 pixels wide)
	//
	//   #
	//  #
	//  #
	// #
	// #
	// #
	// #
	// #
	// #
	//  #
	//  #
	//   #
	//
	//
	0xF0, 0x0C, 0x02,
	0x03, 0x0C, 0x10,

	// @80 ')' (3 pixels wide)
	//
	// #
	//  #
	//  #
	//   #
	//   #
	//   #
	//   #
	//   #
	//   #
	//  #
	//  #
	// #
	//
	//
	0x02, 0x0C, 0xF0,
	0x10, 0x0C, 0x03,

	// @86 '*' (7 pixels wide)
	//
	//    #
	//    #
	// #######
	//    #
	//   # #
	//  #   #
	//
	//
	//
	//
	//
	//
	//
	//
	0x08, 0x48, 0x28, 0x1E, 0x28, 0x48, 0x08,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	// @100 '+' (7 pixels wide)
	//
	//
	//    #
	//    #
	//    #
	//    #
	// #######
	//    #
	//    #
	//    #
	//    #
	//
	//
	//
	//
	0x40, 0x40, 0x40, 0xFC, 0x40, 0x40, 0x40,
	0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00,

	// @114 ',' (3 pixels wide)
	//
	//
	//
	//
	//
	//
	//
	//
	//  ##
	//  #
	// ##
	// #
	// #
	//
	//
	0x00, 0x00, 0x00,
	0x1C, 0x07, 0x01,

	// @120 '-' (7 pixels wide)
	//
	//
	//
	//
	//
	//
	// #######
	//
	//
	//
	//
	//
	//
	//
	//
	0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	// @134 '.' (2 pixels wide)
	//
	//
	//
	//
	//
	//
	//
	//
	//
	// ##
	// ##
	//
	//
	//
	//
	0x00, 0x00,
	0x06, 0x06,

	// @138 '/' (6 pixels wide)
	//      #
	//      #
	//     #
	//     #
	//    #
	//    #
	//   #
	//   #
	//  #
	//  #
	// #
	// #
	//
	//
	//
	0x00, 0x00, 0xC0, 0x30, 0x0C, 0x03,
	0x0C, 0x03, 0x00, 0x00, 0x00, 0x00,

	// @150 '0' (6 pixels wide)
	//
	//  ####
	// #    #
	// #    #
	// #    #
	// #    #
	// #    #
	// #    #
	// #    #
	// #    #
	//  ####
	//
	//
	//
	//
	0xFC, 0x02, 0x02, 0x02, 0x02, 0xFC,
	0x03, 0x04, 0x04, 0x04, 0x04, 0x03,

	// @162 '1' (7 pixels wide)
	//
	//   ##
	// ## #
	//    #
	//    #
	//    #
	//    #
	//    #
	//    #
	//    #
	// #######
	//
	//
	//
	//
	0x04, 0x04, 0x02, 0xFE, 0x00, 0x00, 0x00,
	0x04, 0x04, 0x04, 0x07, 0x04, 0x04, 0x04,

	// @176 '2' (7 pixels wide)
	//
	//  #####
	// #     #
	// #     #
	//       #
	//      #
	//    ##
	//   #
	//  #
	// #     #
	// #######
	//
	//
	//
	//
	0x0C, 0x02, 0x82, 0x42, 0x42, 0x22, 0x1C,
	0x06, 0x05, 0x04, 0x04, 0x04, 0x04, 0x06,

	// @190 '3' (7 pixels wide)
	//
	//   ####
	// ##    #
	//       #
	//       #
	//    ###
	//      #
	//       #
	//       #
	// #     #
	//  #####
	//
	//
	//
	//
	0x04, 0x04, 0x02, 0x22, 0x22, 0x62, 0x9C,
	0x02, 0x04, 0x04, 0x04, 0x04, 0x04, 0x03,

	// @204 '4' (6 pixels wide)
	//
	//     #
	//    ##
	//   # #
	//  #  #
	//  #  #
	// #   #
	// ######
	//     #
	//     #
	//   ####
	//
	//
	//
	//
	0xC0, 0xB0, 0x88, 0x84, 0xFE, 0x80,
	0x00, 0x00, 0x04, 0x04, 0x07, 0x04,

	// @216 '5' (7 pixels wide)
	//
	//  #####
	//  #
	//  #
	//  #
	//  #####
	//       #
	//       #
	//       #
	// #     #
	//  #####
	//
	//
	//
	//
	0x00, 0x3E, 0x22, 0x22, 0x22, 0x22, 0xC0,
	0x02, 0x04, 0x04, 0x04, 0x04, 0x04, 0x03,

	// @230 '6' (7 pixels wide)
	//
	//    ####
	//   #
	//  #
	// #
	// # ####
	// ##    #
	// #     #
	// #     #
	// #     #
	//  #####
	//
	//
	//
	//
	0xF0, 0x48, 0x24, 0x22, 0x22, 0x22, 0xC2,
	0x03, 0x04, 0x04, 0x04, 0x04, 0x04, 0x03,

	// @244 '7' (7 pixels wide)
	//
	// #######
	// #     #
	//       #
	//      #
	//      #
	//     #
	//     #
	//     #
	//    #
	//    #
	//
	//
	//
	//
	0x06, 0x02, 0x02, 0x02, 0xC2, 0x32, 0x0E,
	0x00, 0x00, 0x00, 0x06, 0x01, 0x00, 0x00,

	// @258 '8' (6 pixels wide)
	//
	//  ####
	// #    #
	// #    #
	// #    #
	//  ####
	// #    #
	// #    #
	// #    #
	// #    #
	//  ####
	//
	//
	//
	//
	0xDC, 0x22, 0x22, 0x22, 0x22, 0xDC,
	0x03, 0x04, 0x04, 0x04, 0x04, 0x03,

	// @270 '9' (6 pixels wide)
	//
	//  ####
	// #    #
	// #    #
	// #    #
	// #   ##
	//  ### #
	//      #
	//     #
	//    #
	// ###
	//
	//
	//
	//
	0x3C, 0x42, 0x42, 0x42, 0x22, 0xFC,
	0x04, 0x04, 0x04, 0x02, 0x01, 0x00,

	// @282 ':' (2 pixels wide)
	//
	//
	//
	//
	// ##
	// ##
	//
	//
	//
	// ##
	// ##
	//
	//
	//
	//
	0x30, 0x30,
	0x06, 0x06,

	// @286 ';' (3 pixels wide)
	//
	//
	//
	//
	//  ##
	//  ##
	//
	//
	//  ##
	//  #
	// ##
	// #
	//
	//
	//
	0x00, 0x30, 0x30,
	0x0C, 0x07, 0x01,

	// @292 '<' (8 pixels wide)
	//
	//
	//       ##
	//      #
	//    ##
	//   #
	// ##
	//   #
	//    ##
	//      #
	//       ##
	//
	//
	//
	//
	0x40, 0x40, 0xA0, 0x10, 0x10, 0x08, 0x04, 0x04,
	0x00, 0x00, 0x00, 0x01, 0x01, 0x02, 0x04, 0x04,

	// @308 '=' (8 pixels wide)
	//
	//
	//
	//
	// ########
	//
	//
	// ########
	//
	//
	//
	//
	//
	//
	//
	0x90, 0x90, 0x90, 0x90, 0x90, 0x90, 0x90, 0x90,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	// @324 '>' (8 pixels wide)
	//
	//
	// ##
	//   #
	//    ##
	//      #
	//       ##
	//      #
	//    ##
	//   #
	// ##
	//
	//
	//
	//
	0x04, 0x04, 0x08, 0x10, 0x10, 0xA0, 0x40, 0x40,
	0x04, 0x04, 0x02, 0x01, 0x01, 0x00, 0x00, 0x00,

	// @340 '?' (6 pixels wide)
	//
	//
	//  ####
	// #    #
	// #    #
	//      #
	//     #
	//    #
	//
	//   ##
	//   ##
	//
	//
	//
	//
	0x18, 0x04, 0x04, 0x84, 0x44, 0x38,
	0x00, 0x00, 0x06, 0x06, 0x00, 0x00,

	// @352 '@' (7 pixels wide)
	//
	//   ####
	//  #    #
	// #     #
	// #   ###
	// #  #  #
	// #  #  #
	// #  #  #
	// #   ###
	// #
	//  #   #
	//   ###
	//
	//
	//
	0xF8, 0x04, 0x02, 0xE2, 0x12, 0x12, 0xFC,
	0x03, 0x04, 0x08, 0x08, 0x09, 0x05, 0x01,

	// @366 'A' (10 pixels wide)
	//
	//
	//    ###
	//     ##
	//    #  #
	//    #  #
	//    #  #
	//   ######
	//   #    #
	//  #      #
	// ###    ###
	//
	//
	//
	//
	0x00, 0x00, 0x80, 0xF4, 0x8C, 0x8C, 0xF0, 0x80, 0x00, 0x00,
	0x04, 0x06, 0x05, 0x00, 0x00, 0x00, 0x00, 0x05, 0x06, 0x04,

	// @386 'B' (7 pixels wide)
	//
	//
	// ######
	//  #    #
	//  #    #
	//  #    #
	//  #####
	//  #    #
	//  #    #
	//  #    #
	// ######
	//
	//
	//
	//
	0x04, 0xFC, 0x44, 0x44, 0x44, 0x44, 0xB8,
	0x04, 0x07, 0x04, 0x04, 0x04, 0x04, 0x03,

	// @400 'C' (8 pixels wide)
	//
	//
	//   #### #
	//  #    ##
	// #      #
	// #
	// #
	// #
	// #
	//  #     #
	//   #####
	//
	//
	//
	//
	0xF0, 0x08, 0x04, 0x04, 0x04, 0x04, 0x08, 0x1C,
	0x01, 0x02, 0x04, 0x04, 0x04, 0x04, 0x04, 0x02,

	// @416 'D' (8 pixels wide)
	//
	//
	// ######
	//  #    #
	//  #     #
	//  #     #
	//  #     #
	//  #     #
	//  #     #
	//  #    #
	// ######
	//
	//
	//
	//
	0x04, 0xFC, 0x04, 0x04, 0x04, 0x04, 0x08, 0xF0,
	0x04, 0x07, 0x04, 0x04, 0x04, 0x04, 0x02, 0x01,

	// @432 'E' (7 pixels wide)
	//
	//
	// #######
	//  #    #
	//  #    #
	//  #  #
	//  ####
	//  #  #
	//  #    #
	//  #    #
	// #######
	//
	//
	//
	//
	0x04, 0xFC, 0x44, 0x44, 0xE4, 0x04, 0x1C,
	0x04, 0x07, 0x04, 0x04, 0x04, 0x04, 0x07,

	// @446 'F' (7 pixels wide)
	//
	//
	// #######
	//  #    #
	//  #    #
	//  #  #
	//  ####
	//  #  #
	//  #
	//  #
	// #####
	//
	//
	//
	//
	0x04, 0xFC, 0x44, 0x44, 0xE4, 0x04, 0x1C,
	0x04, 0x07, 0x04, 0x04, 0x04, 0x00, 0x00,

	// @460 'G' (9 pixels wide)
	//
	//
	//   #### #
	//  #    ##
	// #
	// #
	// #
	// #   #####
	// #      #
	//  #     #
	//   #####
	//
	//
	//
	//
	0xF0, 0x08, 0x04, 0x04, 0x84, 0x84, 0x88, 0x8C, 0x80,
	0x01, 0x02, 0x04, 0x04, 0x04, 0x04, 0x04, 0x03, 0x00,

	// @478 'H' (8 pixels wide)
	//
	//
	// ###  ###
	//  #    #
	//  #    #
	//  #    #
	//  ######
	//  #    #
	//  #    #
	//  #    #
	// ###  ###
	//
	//
	//
	//
	0x04, 0xFC, 0x44, 0x40, 0x40, 0x44, 0xFC, 0x04,
	0x04, 0x07, 0x04, 0x00, 0x00, 0x04, 0x07, 0x04,

	// @494 'I' (7 pixels wide)
	//
	//
	// #######
	//    #
	//    #
	//    #
	//    #
	//    #
	//    #
	//    #
	// #######
	//
	//
	//
	//
	0x04, 0x04, 0x04, 0xFC, 0x04, 0x04, 0x04,
	0x04, 0x04, 0x04, 0x07, 0x04, 0x04, 0x04,

	// @508 'J' (8 pixels wide)
	//
	//
	//   ######
	//      #
	//      #
	//      #
	//      #
	// #    #
	// #    #
	// #    #
	//  ####
	//
	//
	//
	//
	0x80, 0x00, 0x04, 0x04, 0x04, 0xFC, 0x04, 0x04,
	0x03, 0x04, 0x04, 0x04, 0x04, 0x03, 0x00, 0x00,

	// @524 'K' (8 pixels wide)
	//
	//
	// ### ####
	//  #   #
	//  #  #
	//  # #
	//  ####
	//  #   #
	//  #   #
	//  #    #
	// ###   ##
	//
	//
	//
	//
	0x04, 0xFC, 0x44, 0x60, 0x54, 0x8C, 0x04, 0x04,
	0x04, 0x07, 0x04, 0x00, 0x00, 0x01, 0x06, 0x04,

	// @540 'L' (8 pixels wide)
	//
	//
	// #####
	//   #
	//   #
	//   #
	//   #
	//   #    #
	//   #    #
	//   #    #
	// ########
	//
	//
	//
	//
	0x04, 0x04, 0xFC, 0x04, 0x04, 0x00, 0x00, 0x80,
	0x04, 0x04, 0x07, 0x04, 0x04, 0x04, 0x04, 0x07,

	// @556 'M' (9 pixels wide)
	//
	//
	// ###   ###
	//  ##   ##
	//  # # # #
	//  # # # #
	//  # # # #
	//  #  #  #
	//  #     #
	//  #     #
	// ###   ###
	//
	//
	//
	//
	0x04, 0xFC, 0x0C, 0x70, 0x80, 0x70, 0x0C, 0xFC, 0x04,
	0x04, 0x07, 0x04, 0x00, 0x00, 0x00, 0x04, 0x07, 0x04,

	// @574 'N' (9 pixels wide)
	//
	//
	// ###   ###
	//  ##    #
	//  # #   #
	//  # #   #
	//  #  #  #
	//  #   # #
	//  #   # #
	//  #    ##
	// ###   ##
	//
	//
	//
	//
	0x04, 0xFC, 0x0C, 0x30, 0x40, 0x80, 0x04, 0xFC, 0x04,
	0x04, 0x07, 0x04, 0x00, 0x00, 0x01, 0x06, 0x07, 0x00,

	// @592 'O' (8 pixels wide)
	//
	//
	//   ####
	//  #    #
	// #      #
	// #      #
	// #      #
	// #      #
	// #      #
	//  #    #
	//   ####
	//
	//
	//
	//
	0xF0, 0x08, 0x04, 0x04, 0x04, 0x04, 0x08, 0xF0,
	0x01, 0x02, 0x04, 0x04, 0x04, 0x04, 0x02, 0x01,

	// @608 'P' (7 pixels wide)
	//
	//
	// ######
	//  #    #
	//  #    #
	//  #    #
	//  #    #
	//  #####
	//  #
	//  #
	// #####
	//
	//
	//
	//
	0x04, 0xFC, 0x84, 0x84, 0x84, 0x84, 0x78,
	0x04, 0x07, 0x04, 0x04, 0x04, 0x00, 0x00,

	// @622 'Q' (8 pixels wide)
	//
	//
	//   ####
	//  #    #
	// #      #
	// #      #
	// #      #
	// #      #
	// #      #
	//  #    #
	//   ####
	//    #####
	//   #
	//
	//
	0xF0, 0x08, 0x04, 0x04, 0x04, 0x04, 0x08, 0xF0,
	0x01, 0x02, 0x14, 0x0C, 0x0C, 0x0C, 0x0A, 0x09,

	// @638 'R' (8 pixels wide)
	//
	//
	// ######
	//  #    #
	//  #    #
	//  #    #
	//  #####
	//  #  #
	//  #   #
	//  #    #
	// ###   ##
	//
	//
	//
	//
	0x04, 0xFC, 0x44, 0x44, 0xC4, 0x44, 0x38, 0x00,
	0x04, 0x07, 0x04, 0x00, 0x00, 0x01, 0x06, 0x04,

	// @654 'S' (6 pixels wide)
	//
	//
	//  ### #
	// #   ##
	// #    #
	// #
	//  ####
	//      #
	// #    #
	// ##   #
	// # ###
	//
	//
	//
	//
	0x38, 0x44, 0x44, 0x44, 0x48, 0x9C,
	0x07, 0x02, 0x04, 0x04, 0x04, 0x03,

	// @666 'T' (7 pixels wide)
	//
	//
	// #######
	// #  #  #
	// #  #  #
	// #  #  #
	//    #
	//    #
	//    #
	//    #
	//  #####
	//
	//
	//
	//
	0x3C, 0x04, 0x04, 0xFC, 0x04, 0x04, 0x3C,
	0x00, 0x04, 0x04, 0x07, 0x04, 0x04, 0x00,

	// @680 'U' (8 pixels wide)
	//
	//
	// ###  ###
	//  #    #
	//  #    #
	//  #    #
	//  #    #
	//  #    #
	//  #    #
	//  #    #
	//   ####
	//
	//
	//
	//
	0x04, 0xFC, 0x04, 0x00, 0x00, 0x04, 0xFC, 0x04,
	0x00, 0x03, 0x04, 0x04, 0x04, 0x04, 0x03, 0x00,

	// @696 'V' (10 pixels wide)
	//
	//
	// ###    ###
	//  #      #
	//   #    #
	//   #    #
	//    #  #
	//    #  #
	//    #  #
	//     ##
	//     ##
	//
	//
	//
	//
	0x04, 0x0C, 0x34, 0xC0, 0x00, 0x00, 0xC0, 0x34, 0x0C, 0x04,
	0x00, 0x00, 0x00, 0x01, 0x06, 0x06, 0x01, 0x00, 0x00, 0x00,

	// @716 'W' (9 pixels wide)
	//
	//
	// #### ####
	//  #     #
	//  #  #  #
	//  #  #  #
	//  # # # #
	//  # # # #
	//  # # # #
	//  # # # #
	//   #   #
	//
	//
	//
	//
	0x04, 0xFC, 0x04, 0xC4, 0x30, 0xC4, 0x04, 0xFC, 0x04,
	0x00, 0x03, 0x04, 0x03, 0x00, 0x03, 0x04, 0x03, 0x00,

	// @734 'X' (9 pixels wide)
	//
	//
	// ###   ###
	//  #     #
	//   #   #
	//    # #
	//     #
	//    # #
	//   #   #
	//  #     #
	// ###   ###
	//
	//
	//
	//
	0x04, 0x0C, 0x14, 0xA0, 0x40, 0xA0, 0x14, 0x0C, 0x04,
	0x04, 0x06, 0x05, 0x00, 0x00, 0x00, 0x05, 0x06, 0x04,

	// @752 'Y' (9 pixels wide)
	//
	//
	// ###   ###
	//  #     #
	//   #   #
	//    # #
	//     #
	//     #
	//     #
	//     #
	//   #####
	//
	//
	//
	//
	0x04, 0x0C, 0x14, 0x20, 0xC0, 0x20, 0x14, 0x0C, 0x04,
	0x00, 0x00, 0x04, 0x04, 0x07, 0x04, 0x04, 0x00, 0x00,

	// @770 'Z' (6 pixels wide)
	//
	//
	// ######
	// #    #
	//     #
	//    #
	//    #
	//   #
	//  #   #
	// #    #
	// ######
	//
	//
	//
	//
	0x0C, 0x04, 0x84, 0x64, 0x14, 0x0C,
	0x06, 0x05, 0x04, 0x04, 0x04, 0x07,

	// @782 '[' (3 pixels wide)
	//
	// ###
	// #
	// #
	// #
	// #
	// #
	// #
	// #
	// #
	// #
	// #
	// ###
	//
	//
	0xFE, 0x02, 0x02,
	0x1F, 0x10, 0x10,

	// @788 '\' (6 pixels wide)
	// #
	// #
	//  #
	//  #
	//   #
	//   #
	//   #
	//    #
	//    #
	//     #
	//     #
	//      #
	//
	//
	//
	0x03, 0x0C, 0x70, 0x80, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x01, 0x06, 0x08,

	// @800 ']' (3 pixels wide)
	//
	// ###
	//   #
	//   #
	//   #
	//   #
	//   #
	//   #
	//   #
	//   #
	//   #
	//   #
	// ###
	//
	//
	0x02, 0x02, 0xFE,
	0x10, 0x10, 0x1F,

	// @806 '^' (7 pixels wide)
	//    #
	//    #
	//   # #
	//  #   #
	// #     #
	//
	//
	//
	//
	//
	//
	//
	//
	//
	//
	0x10, 0x08, 0x04, 0x03, 0x04, 0x08, 0x10,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	// @820 '_' (10 pixels wide)
	//
	//
	//
	//
	//
	//
	//
	//
	//
	//
	//
	//
	//
	//
	// ##########
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40,

	// @840 '`' (3 pixels wide)
	//
	// #
	//  #
	//   #
	//
	//
	//
	//
	//
	//
	//
	//
	//
	//
	//
	0x02, 0x04, 0x08,
	0x00, 0x00, 0x00,

	// @846 'a' (8 pixels wide)
	//
	//
	//
	//
	//   ####
	//  #    #
	//       #
	//  ######
	// #     #
	// #    ##
	//  #### ##
	//
	//
	//
	//
	0x00, 0xA0, 0x90, 0x90, 0x90, 0x90, 0xE0, 0x00,
	0x03, 0x04, 0x04, 0x04, 0x04, 0x02, 0x07, 0x04,

	// @862 'b' (8 pixels wide)
	//
	// ##
	//  #
	//  #
	//  # ###
	//  ##   #
	//  #     #
	//  #     #
	//  #     #
	//  ##   #
	// ## ###
	//
	//
	//
	//
	0x02, 0xFE, 0x20, 0x10, 0x10, 0x10, 0x20, 0xC0,
	0x04, 0x07, 0x02, 0x04, 0x04, 0x04, 0x02, 0x01,

	// @878 'c' (8 pixels wide)
	//
	//
	//
	//
	//   #### #
	//  #    ##
	// #
	// #
	// #
	//  #     #
	//   #####
	//
	//
	//
	//
	0xC0, 0x20, 0x10, 0x10, 0x10, 0x10, 0x20, 0x30,
	0x01, 0x02, 0x04, 0x04, 0x04, 0x04, 0x04, 0x02,

	// @894 'd' (8 pixels wide)
	//
	//      ##
	//       #
	//       #
	//   ### #
	//  #   ##
	// #     #
	// #     #
	// #     #
	//  #   ##
	//   ### ##
	//
	//
	//
	//
	0xC0, 0x20, 0x10, 0x10, 0x10, 0x22, 0xFE, 0x00,
	0x01, 0x02, 0x04, 0x04, 0x04, 0x02, 0x07, 0x04,

	// @910 'e' (7 pixels wide)
	//
	//
	//
	//
	//   ###
	//  #   #
	// #     #
	// #######
	// #
	//  #    #
	//   ####
	//
	//
	//
	//
	0xC0, 0xA0, 0x90, 0x90, 0x90, 0xA0, 0xC0,
	0x01, 0x02, 0x04, 0x04, 0x04, 0x04, 0x02,

	// @924 'f' (8 pixels wide)
	//
	//    #####
	//   #
	//   #
	// #######
	//   #
	//   #
	//   #
	//   #
	//   #
	//  #####
	//
	//
	//
	//
	0x10, 0x10, 0xFC, 0x12, 0x12, 0x12, 0x12, 0x02,
	0x00, 0x04, 0x07, 0x04, 0x04, 0x04, 0x00, 0x00,

	// @940 'g' (8 pixels wide)
	//
	//
	//
	//
	//   ### ##
	//  #   ##
	// #     #
	// #     #
	// #     #
	//  #   ##
	//   ### #
	//       #
	//       #
	//  #####
	//
	0xC0, 0x20, 0x10, 0x10, 0x10, 0x20, 0xF0, 0x10,
	0x01, 0x22, 0x24, 0x24, 0x24, 0x22, 0x1F, 0x00,

	// @956 'h' (8 pixels wide)
	//
	// ##
	//  #
	//  #
	//  # ###
	//  ##   #
	//  #    #
	//  #    #
	//  #    #
	//  #    #
	// ###  ###
	//
	//
	//
	//
	0x02, 0xFE, 0x20, 0x10, 0x10, 0x10, 0xE0, 0x00,
	0x04, 0x07, 0x04, 0x00, 0x00, 0x04, 0x07, 0x04,

	// @972 'i' (7 pixels wide)
	//    #
	//    #
	//
	//
	//  ###
	//    #
	//    #
	//    #
	//    #
	//    #
	// #######
	//
	//
	//
	//
	0x00, 0x10, 0x10, 0xF3, 0x00, 0x00, 0x00,
	0x04, 0x04, 0x04, 0x07, 0x04, 0x04, 0x04,

	// @986 'j' (6 pixels wide)
	//     #
	//     #
	//
	//
	//  #####
	//      #
	//      #
	//      #
	//      #
	//      #
	//      #
	//      #
	//      #
	// #####
	//
	0x00, 0x10, 0x10, 0x10, 0x13, 0xF0,
	0x20, 0x20, 0x20, 0x20, 0x20, 0x1F,

	// @998 'k' (8 pixels wide)
	//
	// ##
	//  #
	//  #
	//  #  ###
	//  #  #
	//  # #
	//  ###
	//  #  #
	//  #   #
	// ##  ####
	//
	//
	//
	//
	0x02, 0xFE, 0x80, 0xC0, 0x30, 0x10, 0x10, 0x00,
	0x04, 0x07, 0x00, 0x00, 0x05, 0x06, 0x04, 0x04,

	// @1014 'l' (7 pixels wide)
	//
	//  ###
	//    #
	//    #
	//    #
	//    #
	//    #
	//    #
	//    #
	//    #
	// #######
	//
	//
	//
	//
	0x00, 0x02, 0x02, 0xFE, 0x00, 0x00, 0x00,
	0x04, 0x04, 0x04, 0x07, 0x04, 0x04, 0x04,

	// @1028 'm' (9 pixels wide)
	//
	//
	//
	//
	// ## #  #
	//  ## ## #
	//  #  #  #
	//  #  #  #
	//  #  #  #
	//  #  #  #
	// ### ## ##
	//
	//
	//
	//
	0x10, 0xF0, 0x20, 0x10, 0xE0, 0x20, 0x10, 0xE0, 0x00,
	0x04, 0x07, 0x04, 0x00, 0x07, 0x04, 0x00, 0x07, 0x04,

	// @1046 'n' (8 pixels wide)
	//
	//
	//
	//
	// ## ###
	//  ##   #
	//  #    #
	//  #    #
	//  #    #
	//  #    #
	// ###  ###
	//
	//
	//
	//
	0x10, 0xF0, 0x20, 0x10, 0x10, 0x10, 0xE0, 0x00,
	0x04, 0x07, 0x04, 0x00, 0x00, 0x04, 0x07, 0x04,

	// @1062 'o' (7 pixels wide)
	//
	//
	//
	//
	//   ###
	//  #   #
	// #     #
	// #     #
	// #     #
	//  #   #
	//   ###
	//
	//
	//
	//
	0xC0, 0x20, 0x10, 0x10, 0x10, 0x20, 0xC0,
	0x01, 0x02, 0x04, 0x04, 0x04, 0x02, 0x01,

	// @1076 'p' (8 pixels wide)
	//
	//
	//
	//
	// ## ###
	//  ##   #
	//  #     #
	//  #     #
	//  #     #
	//  ##   #
	//  # ###
	//  #
	//  #
	// ###
	//
	0x10, 0xF0, 0x20, 0x10, 0x10, 0x10, 0x20, 0xC0,
	0x20, 0x3F, 0x22, 0x04, 0x04, 0x04, 0x02, 0x01,

	// @1092 'q' (8 pixels wide)
	//
	//
	//
	//
	//   ### ##
	//  #   ##
	// #     #
	// #     #
	// #     #
	//  #   ##
	//   ### #
	//       #
	//       #
	//      ###
	//
	0xC0, 0x20, 0x10, 0x10, 0x10, 0x20, 0xF0, 0x10,
	0x01, 0x02, 0x04, 0x04, 0x04, 0x22, 0x3F, 0x20,

	// @1108 'r' (7 pixels wide)
	//
	//
	//
	//
	// ##  ###
	//  ###
	//  #
	//  #
	//  #
	//  #
	// #####
	//
	//
	//
	//
	0x10, 0xF0, 0x20, 0x20, 0x10, 0x10, 0x10,
	0x04, 0x07, 0x04, 0x04, 0x04, 0x00, 0x00,

	// @1122 's' (7 pixels wide)
	//
	//
	//
	//
	//  #### #
	// #    ##
	// #
	//  #####
	//       #
	// #     #
	// ######
	//
	//
	//
	//
	0x60, 0x90, 0x90, 0x90, 0x90, 0xA0, 0x30,
	0x06, 0x04, 0x04, 0x04, 0x04, 0x04, 0x03,

	// @1136 't' (8 pixels wide)
	//
	//
	//   #
	//   #
	// #######
	//   #
	//   #
	//   #
	//   #
	//   #    #
	//    ####
	//
	//
	//
	//
	0x10, 0x10, 0xFC, 0x10, 0x10, 0x10, 0x10, 0x00,
	0x00, 0x00, 0x03, 0x04, 0x04, 0x04, 0x04, 0x02,

	// @1152 'u' (8 pixels wide)
	//
	//
	//
	//
	// ##   ##
	//  #    #
	//  #    #
	//  #    #
	//  #    #
	//  #   ##
	//   ### ##
	//
	//
	//
	//
	0x10, 0xF0, 0x00, 0x00, 0x00, 0x10, 0xF0, 0x00,
	0x00, 0x03, 0x04, 0x04, 0x04, 0x02, 0x07, 0x04,

	// @1168 'v' (8 pixels wide)
	//
	//
	//
	//
	// ###  ###
	//  #    #
	//   #  #
	//   #  #
	//   #  #
	//    ##
	//    ##
	//
	//
	//
	//
	0x10, 0x30, 0xD0, 0x00, 0x00, 0xD0, 0x30, 0x10,
	0x00, 0x00, 0x01, 0x06, 0x06, 0x01, 0x00, 0x00,

	// @1184 'w' (10 pixels wide)
	//
	//
	//
	//
	// ###     ##
	//  #   #   #
	//  #   #   #
	//   # # # #
	//   # # # #
	//   # # # #
	//    #   #
	//
	//
	//
	//
	0x10, 0x70, 0x90, 0x00, 0x80, 0x60, 0x80, 0x00, 0x90, 0x70,
	0x00, 0x00, 0x03, 0x04, 0x03, 0x00, 0x03, 0x04, 0x03, 0x00,

	// @1204 'x' (8 pixels wide)
	//
	//
	//
	//
	// ###  ###
	//  #    #
	//   #  #
	//    ##
	//   #  #
	//  #    #
	// ###  ###
	//
	//
	//
	//
	0x10, 0x30, 0x50, 0x80, 0x80, 0x50, 0x30, 0x10,
	0x04, 0x06, 0x05, 0x00, 0x00, 0x05, 0x06, 0x04,

	// @1220 'y' (9 pixels wide)
	//
	//
	//
	//
	// ###   ###
	//  #     #
	//   #   #
	//   #   #
	//    # #
	//    # #
	//     #
	//     #
	//    #
	// #####
	//
	0x10, 0x30, 0xD0, 0x00, 0x00, 0x00, 0xD0, 0x30, 0x10,
	0x20, 0x20, 0x20, 0x33, 0x2C, 0x03, 0x00, 0x00, 0x00,

	// @1238 'z' (7 pixels wide)
	//
	//
	//
	//
	// #######
	// #    #
	//     #
	//    #
	//   #
	//  #    #
	// #######
	//
	//
	//
	//
	0x30, 0x10, 0x10, 0x90, 0x50, 0x30, 0x10,
	0x04, 0x06, 0x05, 0x04, 0x04, 0x04, 0x06,

	// @1252 '{' (3 pixels wide)
	//
	//
	//   #
	//  #
	//  #
	//  #
	//  #
	// #
	//  #
	//  #
	//  #
	//  #
	//   #
	//
	//
	0x80, 0x78, 0x04,
	0x00, 0x0F, 0x10,

	// @1258 '|' (1 pixels wide)
	//
	// #
	// #
	// #
	// #
	// #
	// #
	// #
	// #
	// #
	// #
	// #
	// #
	//
	//
	0xFE,
	0x1F,

	// @1260 '}' (3 pixels wide)
	//
	//
	// #
	//  #
	//  #
	//  #
	//  #
	//   #
	//  #
	//  #
	//  #
	//  #
	// #
	//
	//
	0x04, 0x78, 0x80,
	0x10, 0x0F, 0x00,

	// @1266 '~' (7 pixels wide)
	//
	//
	//
	//
	//
	//  ##
	// #  #  #
	//     ##
	//
	//
	//
	//
	//
	//
	//
	0x40, 0x20, 0x20, 0x40, 0x80, 0x80, 0x40,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

// Character descriptors for Courier New 12pt
// { [Char width in bits], [Offset into courierNew_12ptCharBitmaps in bytes] }
static constexpr periph::display::font_t::chr_desc_t courierNew_12ptDescriptors[] =
{
	{3, 0}, 		// !
	{5, 6}, 		// "
	{8, 16}, 		// #
	{6, 32}, 		// $
	{6, 44}, 		// %
	{6, 56}, 		// &
	{3, 68}, 		// '
	{3, 74}, 		// (
	{3, 80}, 		// )
	{7, 86}, 		// *
	{7, 100}, 		// +
	{3, 114}, 		// ,
	{7, 120}, 		// -
	{2, 134}, 		// .
	{6, 138}, 		// /
	{6, 150}, 		// 0
	{7, 162}, 		// 1
	{7, 176}, 		// 2
	{7, 190}, 		// 3
	{6, 204}, 		// 4
	{7, 216}, 		// 5
	{7, 230}, 		// 6
	{7, 244}, 		// 7
	{6, 258}, 		// 8
	{6, 270}, 		// 9
	{2, 282}, 		// :
	{3, 286}, 		// ;
	{8, 292}, 		// <
	{8, 308}, 		// =
	{8, 324}, 		// >
	{6, 340}, 		// ?
	{7, 352}, 		// @
	{10, 366}, 		// A
	{7, 386}, 		// B
	{8, 400}, 		// C
	{8, 416}, 		// D
	{7, 432}, 		// E
	{7, 446}, 		// F
	{9, 460}, 		// G
	{8, 478}, 		// H
	{7, 494}, 		// I
	{8, 508}, 		// J
	{8, 524}, 		// K
	{8, 540}, 		// L
	{9, 556}, 		// M
	{9, 574}, 		// N
	{8, 592}, 		// O
	{7, 608}, 		// P
	{8, 622}, 		// Q
	{8, 638}, 		// R
	{6, 654}, 		// S
	{7, 666}, 		// T
	{8, 680}, 		// U
	{10, 696}, 		// V
	{9, 716}, 		// W
	{9, 734}, 		// X
	{9, 752}, 		// Y
	{6, 770}, 		// Z
	{3, 782}, 		// [
	{6, 788}, 		//
	{3, 800}, 		// ]
	{7, 806}, 		// ^
	{10, 820}, 		// _
	{3, 840}, 		// `
	{8, 846}, 		// a
	{8, 862}, 		// b
	{8, 878}, 		// c
	{8, 894}, 		// d
	{7, 910}, 		// e
	{8, 924}, 		// f
	{8, 940}, 		// g
	{8, 956}, 		// h
	{7, 972}, 		// i
	{6, 986}, 		// j
	{8, 998}, 		// k
	{7, 1014}, 		// l
	{9, 1028}, 		// m
	{8, 1046}, 		// n
	{7, 1062}, 		// o
	{8, 1076}, 		// p
	{8, 1092}, 		// q
	{7, 1108}, 		// r
	{7, 1122}, 		// s
	{8, 1136}, 		// t
	{8, 1152}, 		// u
	{8, 1168}, 		// v
	{10, 1184}, 		// w
	{8, 1204}, 		// x
	{9, 1220}, 		// y
	{7, 1238}, 		// z
	{3, 1252}, 		// {
	{1, 1258}, 		// |
	{3, 1260}, 		// }
	{7, 1266}, 		// ~
};


// Font information for Courier New 12pt
const periph::display::font_t font_default =
{
	15, //  Character height
	'!', //  Start character
	'~', //  End character
	2, //  Width, in pixels, of space character
	courierNew_12ptDescriptors, //  Character descriptor array
	courierNew_12ptBitmaps, //  Character bitmap array
};


}}
