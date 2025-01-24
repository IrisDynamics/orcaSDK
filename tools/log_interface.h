#pragma once

#include <string>
#include "../src/error_types.h"

namespace orcaSDK
{

/**
 *	@brief	An interface that Actuator expects of log implementations.
 */
class LogInterface
{
public:
	/**
	 *	@brief	Create and/or open a file given by path for writing.
	 */
	virtual OrcaError open(const std::string& path) = 0;
	/**
	 *	@brief	Write a message to an openend log file.
	 */
	virtual OrcaError write(const std::string& str) = 0;
};

}