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
#include <string>
#include <type_traits>
 
namespace fnd {
namespace bus {
	class ibus;
} }
 
namespace app {
 
struct envid { // Application config defines
enum _envid {
	dummy,
}; };
 
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
 * @return Error code
 */
int setenv( unsigned env_id, const void* buf, size_t buf_len );

template <typename T> inline int setenv( unsigned env_id, const T& value ) {
	static_assert( std::is_trivial<T>::value, "Only POD type is allowed" );
	return setenv( env_id, &value, sizeof( T ) );
}
 
/** Get environment variable
 * @param[in] env_id Environment identifier
 * @param[in] buf input buffer
 * @param[in] buf_len Buffer size
 * @return Error code
 */
inline int setenv( unsigned env_id, const std::string& str, size_t max_len ) {
	if( max_len > str.size() ) {
		max_len = str.size();
	}
	return setenv( env_id, str.c_str(), max_len );
}
 
/** Get environment variable
 * @param[in] env_id Environment identifier
 * @param[out] buf Output buffer
 * @param[in] buf_len Buffer size
 * @return Current number of bytes read or negative error code
 */
int getenv( unsigned env_id, void* buf, size_t buf_len );

template <typename T> inline int getenv( unsigned env_id, T& value ) {
	static_assert( std::is_trivial<T>::value, "Only POD type is allowed" );
	return getenv( env_id, &value, sizeof( T ) );
}
 
/** Get environment and put it into the string
 * @param[in] env_id Environment identifier
 * @param[out] buf_str String buffera
 * @param[in] max_size String max sizea
 */
inline int getenv( unsigned env_id, std::string& buf_str, size_t max_len ) {
	buf_str.resize( max_len );
	int ret = getenv( env_id, &buf_str[0], max_len );
	if( ret > 0 ) {
		buf_str.resize( ret );
	}
	return ret;
}
 
/** Unset env
 * @param[in] env_id Environment id
 * @return Current status
 */
int unsetenv( unsigned env_id );
 
/** Delete all envirnment variables
 *  and reformat filesystem */
int clearenv( );
 
}
