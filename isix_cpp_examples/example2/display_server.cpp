/* ------------------------------------------------------------------ */
/*
 * display_server.cpp
 * The display server class communicate implementation
 *  Created on: 2010-01-03
 *      Author: lucck
 */
/* ------------------------------------------------------------------ */
#include "display_server.hpp"

/* ------------------------------------------------------------------ */
//Application namespace
namespace app
{
/* ------------------------------------------------------------------ */
//Constructor
display_server::display_server()
	: io_fifo(QUEUE_CAPACITY)
{

}
/* ------------------------------------------------------------------ */
//Main thread
void display_server::main()
{
	const display_msg *msg = NULL;

	for(;;)
	{
		//Try read message from the fifo
		if(io_fifo.pop(msg) == isix::ISIX_EOK)
		{
			//Validate message
			if( msg )
			{
				//Clear display
				display.clear();
				//If text message
				if(msg->get_type()==display_msg::MSG_TEXT)
				{
					//Cast to the text message class
					const text_msg *t = static_cast<const text_msg*>(msg);
					if(t->get_text())
					{
						//Write text on the display
						display.put_string(t->get_text(),t->get_x(),t->get_y());
					}
				}
				//If graphics message
				else if(msg->get_type()==display_msg::MSG_GRAPHICS)
				{
					//Get image
					const images::img_def *img = static_cast<const graph_msg*>(msg)->get_image();
					if(img)
					{
						//Display bitmap
						display.put_bitmap(*img);
					}
				}
			}
		}
	}
}

/* ------------------------------------------------------------------ */
}
/* ------------------------------------------------------------------ */
