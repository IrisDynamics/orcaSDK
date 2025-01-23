#pragma once

#include "clock.h"
#include <chrono>

namespace orcaSDK
{

class ChronoClock : public Clock
{
public:
	ChronoClock() :
		start_time(std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now()))
	{}

	int64_t get_time_microseconds() override
	{
		return (std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now()) - start_time).count();
	}

private:
	std::chrono::time_point<std::chrono::high_resolution_clock, std::chrono::microseconds> start_time;
};

}