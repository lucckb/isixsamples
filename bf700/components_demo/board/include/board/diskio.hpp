/*
 * =====================================================================================
 *
 *       Filename:  diskio.hpp
 *
 *    Description: SD card disk IO operation class
 *
 *        Version:  1.0
 *        Created:  18.07.2014 22:10:28
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Lucjan Bryndza (LB), lucck(at)boff(dot)pl
 *   Organization:  BoFF
 *
 * =====================================================================================
 */
#pragma once

#include <foundation/noncopyable.hpp>
#include <mmc/immc_det_pin.hpp>
#include <mmc/mmc_slot.hpp>
#include <functional>
#include <isix.h>

namespace drv {
namespace disk {

using slot_status = drv::mmc::mmc_slot::status;
using notify_callback = std::function<void(slot_status)>;
static constexpr auto no_block = drv::mmc::mmc_slot::C_no_block;

//! Remount on demand
int remount() noexcept;

//! Initialize DISKIO operation
void init() noexcept;

/**  Disk connect disconnect callback
 *   @param[in] cb Drive notification callback
 */
void connect_notify( notify_callback cb ) noexcept;

/** 
 *  Wait for insert or remove SD disc card
 *  @param[in] timeout Insert timeout
 *  @return Card status @see slot_status
*/
int wait_for_slot_change( ostick_t timeout = ISIX_TIME_INFINITE );

namespace detail {
	class stm32_gpio : public drv::mmc::immc_det_pin {
	public:
		//! Constructor
		stm32_gpio();
		//! Destructor
		virtual ~stm32_gpio() {
		}
		//Get pin method
		virtual bool get() const;
	};
}

}} 

