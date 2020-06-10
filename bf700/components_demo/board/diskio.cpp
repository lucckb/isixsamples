/*
 * =====================================================================================
 *
 *       Filename:  diskio.cpp
 *
 *    Description:  SD card disk IO operation class
 *
 *        Version:  1.0
 *        Created:  18.07.2014 22:21:22
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Lucjan Bryndza (LB), lucck(at)boff(dot)pl
 *   Organization:  BoFF
 *
 * =====================================================================================
 */
#include <board/diskio.hpp>
#include <stm32spi.h>
#include <fs/fat.h>
#include <memory>
#include <atomic>
#include <stm32_sdio_mmc_host.hpp>
#include <mmc/immc_det_pin.hpp>
#include <mmc/mmc_slot.hpp>
#include <mmc/mmc_card.hpp>
#include <foundation/sys/dbglog.h>

namespace drv {
namespace disk {

namespace {
	std::unique_ptr<stm32::drv::mmc_host_sdio> mmc_host;
	std::unique_ptr<detail::stm32_gpio> pin;
	std::unique_ptr<drv::mmc::mmc_slot> slot;
	FATFS fat;
	std::atomic<bool> mounted { };
	notify_callback disk_change_callback;
}

namespace {
	void slot_callback( slot_status st )
	{
		if( st == slot_status::card_removed ) {
			mounted = false;
		}
		if( disk_change_callback ) {
			disk_change_callback( st );
		}
	}
}

//! Initialize DISKIO operation
void init() noexcept
{
	if( !mmc_host ) {
		mmc_host.reset( new stm32::drv::mmc_host_sdio(
			CONFIG_PCLK2_HZ, CONFIG_SDIO_HOST_CLK_KHZ ) );
		pin.reset( new detail::stm32_gpio );
		slot.reset( new drv::mmc::mmc_slot( std::ref(*mmc_host), std::ref(*pin) ) );
		slot->connect( slot_callback );
		disk_add( 0, slot.get() );
	}
}

//! Connect disk state change notify callback
void connect_notify( notify_callback cb ) noexcept
{
	disk_change_callback = cb;
}

//! Get disk fat object
int remount() noexcept
{
	auto err = FR_OK;
	if( !mounted )  {
		err = f_mount( 0 ,&fat );
		dbprintf("Mount res %i", err );
		if( err == FR_OK ) {
			mounted = true;
		}
	}
	return err;
}

//! Wait for slot insert or removal 
int wait_for_slot_change( ostick_t timeout ) 
{
	if( slot ) {
		return slot->check( timeout );
	} else {
		return slot_status::card_removed;
	}
}

namespace detail {
	//! STM32GPIO pin impl
	stm32_gpio::stm32_gpio() {
		using namespace stm32;
		gpio_abstract_config( CONFIG_SD_DET_PORT, CONFIG_SD_DET_PIN, 
				AGPIO_MODE_INPUT_PULLUP, AGPIO_SPEED_HALF );
	}
	bool stm32_gpio::get() const {
		return !stm32::gpio_get( CONFIG_SD_DET_PORT, CONFIG_SD_DET_PIN );
	}
}

}}


