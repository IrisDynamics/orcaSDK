#pragma once

#include "src/clock.h"

class TestClock : public Clock
{
public:
	int64_t get_time_microseconds() override 
	{
		pass_time(auto_time_pass);
		return current_time;
	}

	void pass_time(int64_t new_time)
	{
		current_time += new_time;
	}

	void set_auto_time_pass(int64_t auto_time_pass)
	{
		this->auto_time_pass = auto_time_pass;
	}

private:
	int64_t current_time = 0;
	int64_t auto_time_pass = 0;
};