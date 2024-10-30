#pragma once

#include "pch.h"
#include "tools/log_interface.h"

class TestLog : public LogInterface {
public:
	void open(const std::string&) override {}

	void write(const std::string& str) override
	{
		last_written_string = str;
	}

	std::string last_written_string = "";
};