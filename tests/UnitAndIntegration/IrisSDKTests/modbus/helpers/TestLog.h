#pragma once

#include "pch.h"
#include "tools/log_interface.h"

class TestLog : public LogInterface {
public:
	OrcaError open(const std::string&) override {
		return { false, "" };
	}

	OrcaError write(const std::string& str) override
	{
		last_written_string = str;
		return { false, "" };
	}

	std::string last_written_string = "";
};