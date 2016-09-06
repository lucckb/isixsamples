/*
 * =====================================================================================
 *
 *       Filename:  sdio_tester.cpp
 *
 *    Description:  SDIO tester without class
 *
 *        Version:  1.0
 *        Created:  06.09.2016 18:04:18
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Lucjan Bryndza (LB), lbryndza.p(at)boff(dot)pl
 *   Organization:  BoFF
 *
 * =====================================================================================
 */

#include <fs/fat.h>
#include <board/diskio.hpp>
#include <foundation/dbglog.h>

namespace app {


void stdio_test_task( )
{
	dbg_info("Hello from SDIO test task");
	drv::disk::init();
	for(;;) {
		if( drv::disk::wait_for_slot_change()
			!= drv::disk::slot_status::card_inserted ) {
			continue;
		}
		dbg_info("SDIO card inserted");
		int err = drv::disk::remount();
		dbg_info("SDIO remount code %i", err );
		if( err ) {
			dbg_err("Fatal unable to remount SD card");
			return;
		}
		FIL f;
		err = f_open( &f, "/test.txt", FA_READ|FA_OPEN_EXISTING );
		if( err ) {
			dbg_err("Unable to open file for read %i", err );
			return;
		}
		char buf[64] {};
		unsigned br;
		err = f_read( &f, buf, sizeof(buf)-1, &br );
		dbg_info("Rd status err %i nrd %u [%s]", err, br, buf );
		f_close( &f );
	}
}


}

