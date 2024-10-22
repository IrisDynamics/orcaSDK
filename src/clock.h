#pragma once

#include <cstdint>

class Clock
{
public:
	virtual int64_t get_time_microseconds() = 0;
};