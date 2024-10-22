#pragma once

#include "src/clock.h"

class TestClock : public Clock
{
public:
	int64_t get_time_microseconds() override 
	{
		return current_time;
	}

	void pass_time(uint32_t new_time)
	{
		current_time += new_time;
	}

private:
	int64_t current_time = 0;
};