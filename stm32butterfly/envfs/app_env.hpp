/*
 * =====================================================================================
 *
 *       Filename:  app_env.hpp
 *
 *    Description:  App environment variables
 *
 *        Version:  1.0
 *        Created:  06/24/2014 03:14:25 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Lucjan Bryndza (lb), lucck@boff.pl
 *   Organization:  BoFF
 *
 * =====================================================================================
 */

#pragma once
 
#include <cstddef>
 
namespace fnd {
namespace bus {
	class ibus;
} }
 
namespace app {
  
struct env {
	enum _err {
		err_not_init = -1
	};
};
  
/** Init environment variable stuff
 * @param[in] Internal bus
 * @return error code
 */
void initenv( fnd::bus::ibus& bus );

  
/** Get environment variable
 * @param[in] env_id Environment identifier
 * @param[in] buf Input buffer
 * @param[in] buf_len Buffer size
 * @return Current status
 */
int setenv( unsigned env_id, const void* buf, size_t buf_len );

template <typename T> int setenv( unsigned env_id, const T& value ) {
	return setenv( env_id, &value, sizeof( T ) );
}
  
/** Get environment variable
 * @param[in] env_id Environment identifier
 * @param[out] buf Output buffer
 * @param[in] buf_len Buffer size
 * @return Current status
 */
int getenv(  unsigned env_id, void* buf, size_t buf_len );

template <typename T> int getenv( unsigned env_id, T& value ) {
	return getenv( env_id, &value, sizeof( T ) );
}
  
/** Unset env 
 * @param[in] env_id Environment id
 * @return Current status
 */
int unsetenv( unsigned env_id );
  
}
