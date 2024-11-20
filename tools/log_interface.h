#pragma once

#include <string>
#include "../src/error_types.h"

class LogInterface
{
public:
	virtual OrcaError open(const std::string& path) = 0;
	virtual OrcaError write(const std::string& str) = 0;
};