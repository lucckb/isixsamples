/*
 * manual_icon.cpp
 *
 *  Created on: 2 lis 2013
 *      Author: lucck
 */

#include "resources.hpp"


namespace app {
namespace res {
//
//  Image data for manual
//
//
//  Image data for manual1
//

static constexpr unsigned char manualBitmaps[] =
{
	//       ####
	//   ### #  ####
	//   # ###  #  ###
	//   #  ##  #  # ##
	//   ##  #  #  #  #
	//    #  #  #  #  #
	//  ###        #  #
	// #  ##          #
	// #   #          #
	// ##            ##
	//  #            #
	//   #          ##
	//    #         #
	//     #       ##
	//      #      #
	0x80, 0x40, 0x5E, 0xF2, 0x86, 0x0C, 0x3F, 0x01, 0x01, 0x3F, 0x02, 0x02, 0x7E, 0x04, 0x0C, 0xF8,
	0x03, 0x06, 0x08, 0x10, 0x21, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x38, 0x0E, 0x03,
};


const periph::display::icon_t manual_icon
{
	2,
	16,
	manualBitmaps
};


}}
