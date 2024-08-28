#pragma once

#include <string>

class LogInterface
{
public:
	virtual void write(const std::string& str) = 0;
};