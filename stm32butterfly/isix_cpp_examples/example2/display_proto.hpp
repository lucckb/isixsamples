 
/*
 * display_proto.hpp
 * The display protocol class implements the base msg classes
 *  Created on: 2010-01-09
 *      Author: lucck
 */
 
#ifndef DISPLAY_PROTO_HPP_
#define DISPLAY_PROTO_HPP_
 
//Forward declaration
namespace images
{
	struct img_def;
}

 
//App namespace
namespace app
{
 
//The base class display message
class display_msg
{
public:

	enum msg_type {MSG_GRAPHICS, MSG_TEXT };

	//Default constructor
	display_msg(msg_type _msg):msg(_msg) {}

	//Get type method
	msg_type get_type() const
	{
		return msg;
	}

private:
	msg_type msg;
};

 
//Text message class
class text_msg : public display_msg
{
public:

	//Constructor
	text_msg(const char *_txt=NULL,short _xpos=0,short _ypos=0)
	: display_msg(display_msg::MSG_TEXT),
	  txt(_txt),xpos(_xpos),ypos(_ypos)
	{

	}

	//Set text
	void set_text(const char *_txt, short _xpos=0, short _ypos=0)
	{
		txt = _txt;
		xpos = _xpos;
		ypos = _ypos;
	}

	//Get text
	const char* get_text() const
	{
		return txt;
	}

	//Get position x
	short get_x() const
	{
		return xpos;
	}

	//Get position y
	short get_y() const
	{
		return ypos;
	}

private:
	const char *txt;
	short xpos;
	short ypos;
};
 
//Graphics message class
class graph_msg : public display_msg
{
public:
	//Constructor
	graph_msg(const images::img_def *_img=NULL)
		: display_msg(display_msg::MSG_GRAPHICS),img(_img) {}

	//Set image
	void set_image(const images::img_def &_img) { img = &_img; }

	//Get image
	const images::img_def* get_image() const { return img; }

private:
	const images::img_def *img;
};

 
}

 
#endif /* DISPLAY_PROTO_HPP_ */
 
