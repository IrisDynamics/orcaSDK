#pragma once

#include <string>

class LogInterface
{
public:
	virtual void open(const std::string& path) = 0;
	virtual void write(const std::string& str) = 0;
};