 
/*
 * img_def.hpp
 *
 *  Created on: 2010-01-04
 *      Author: lucck
 */
 
#ifndef IMG_DEF_HPP_
#define IMG_DEF_HPP_
 
//Images namespace
namespace images
{
	//Image definition from pure C a SPGL library.
	struct img_def
	{
		int width;
		int height;
		const unsigned char *data;
	};
}
 
#endif /* IMG_DEF_HPP_ */
