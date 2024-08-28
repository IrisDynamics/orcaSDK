#pragma once

#include "pch.h"
#include "tools/LogInterface.h"

class TestLog : public LogInterface {
public:
	void write(const std::string& str) override
	{
		last_written_string = str;
	}

	std::string last_written_string = "";
};