#pragma once

#include <string>

namespace orcaSDK
{

/**
 *	@brief	The type used to report all errors occurring in the orcaSDK. 
 *
 *		All functions in the SDK that can fail will return an 
 *		instance of either this type, or an OrcaResult<T>, which contains
 *		an instance of this type. To determine if an instance of this type
 *		represents an error or not, it should be converted to a boolean type.
 *		If that conversion evaluates to true, then an error has occurred, and
 *		the details can be found through the OrcaError::what() function. If 
 *		the conversion evaluates to false, then no error has occurred and the
 *		operation that returned this type resulted in a success.
 */
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

	/**
	 *	@brief	Explicit boolean conversion operator. 
	 *		
	 *		Functions that return this type in this SDK may fail. These failures should be
	 *		checked by making use of this conversion operator. Evaluation of
	 *		true means an error is present, and evaluation of false means the
	 *		operation completed successfully.
	 */
	explicit operator bool() const
	{
		return failure;
	}

	/**
	 *	@brief	The error message associated with an error.
	 * 
	 *		In the case of an error, this function will return a string containing 
	 *		a description of what error has occured. If no error has occurred, this
	 *		function will return an empty string.
	 *	@return	std::string	An error message containing a description of what error occurred
	 */
	std::string what() const
	{
		return error_message;
	}

private:

	bool failure;
	std::string error_message;
};

/**
 *	@brief	This struct is returned when a function is expected to return a type, but
 *			the function might fail in some circumstances.
 * 
 *		This struct contains two members: value and error. The value member contains
 *		the type that should be returned in the "happy path" of the function. The 
 *		value member should only be used the the case that a boolean conversion of 
 *		the error member evaluates to false. See the documentation for OrcaError for
 *		more details. The contents of the value member are undefined in the case that
 *		an error is present.
 */
template <typename T>
struct OrcaResult
{
	T value; //!< The resulting value of the operation if the operation completes successfully
	OrcaError error; //!< The error object, which should be checked to determine whether an error has occurred
};

}