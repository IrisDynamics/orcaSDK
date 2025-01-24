#pragma once

#include <cstdint>

namespace orcaSDK
{

/** 
 * @brief	An interface class for implementing time on non-standard systems
 */
class Clock
{
public:
	/**
	 * @brief	Obtain a timestamp for the current time in microseconds.
	 * @return	int64_t - The current time in microseconds.
	 * @note	The zero value for this time can be anything so long as 
	 *			the clock increases roughly monotonically.
	 * @note	The possible range of times must span the full signed 64-bit
	 *			range. If the actual clock used for input to this function uses
	 *			a more restricted data-type (eg 32-bit ints), then such input
	 *			must be transformed to fit this range before being returned.
	 */
	virtual int64_t get_time_microseconds() = 0;
};

}