 
/*
 * display_proto.cpp
 *
 *  Created on: 2010-02-14
 *      Author: lucck
 */
 
#include "display_proto.hpp"


 
namespace app
{
 
void time_msg::strrev(char *str, int len)
{
	char *p1, *p2;

	if (! str || ! *str)
	   return;
	for (p1 = str, p2 = str + len - 1; p2 > p1; ++p1, --p2)
	{
		*p1 ^= *p2;
		*p2 ^= *p1;
		*p1 ^= *p2;
	}
}

 
const char* time_msg::conv_hex(char *txt,unsigned value,int zeros)
{
	if(zeros>7) zeros = 7;
	int c;
	//Convert in reverse order
	for(c=0; value!=0; c++,value /=16)
		txt[c] = value %16 + '0';
	//Add zeros
	for(;c<zeros;c++)
		txt[c] = '0';
	//Move to requested pos
	txt[c] = '\0';
	strrev(txt,c);
	return txt;
}
 
}
 

