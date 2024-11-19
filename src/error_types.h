#pragma once

#include <string>

class OrcaError
{
public:
	OrcaError(
		const int failure_type,
		std::string error_message = ""
	) :
		failure(failure_type),
		error_message(error_message)
	{}
	OrcaError& operator=(const OrcaError& other) = default;

	explicit operator bool() const
	{
		return failure;
	}

	std::string what()
	{
		return error_message;
	}

private:

	bool failure;
	std::string error_message;
};

template <typename T>
struct OrcaResult
{
	T value;
	OrcaError error;
};