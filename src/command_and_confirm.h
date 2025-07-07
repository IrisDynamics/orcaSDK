#pragma once

#include "../actuator.h"

namespace orcaSDK
{

/**
 *	@overload	bool Actuator::command_and_confirm(uint16_t command_register_address, uint16_t command_register_value, uint16_t confirm_register_address, uint16_t confirm_register_value);
 *	@brief		Writes to a register and blocks the current thread until a read register matches a given value.
 *	@param	confirm_register_value	The value that the register in confirm_register_address should have for the command to have been considered a success
*/
[[nodiscard("Ignored failure here will usually lead to an invalid application state")]]
bool command_and_confirm(
	Actuator& motor, 
	uint16_t command_register_address, uint16_t command_register_value, 
	uint16_t confirm_register_address, uint16_t confirm_register_value, 
	const int max_wait_time_ms = 125);

/**
 *	@brief		Writes to a register and blocks the current thread until some post-condition is observed.
 *	@details	Writes to modbus address <command_register_address> with value
 *				<command_register_value> while reading from <confirm_register_address>. 
 *				Will repeatedly perform this write and read while calling <success_function> until it returns a value of true.
 *	@param	command_register_address	The register being written to
 *	@param	command_register_value		The value to be written
 *	@param	confirm_register_address	The register that should be read from for confirmation
 *	@param	success_function			The function that must return true for the command to have been considered a success
*/
[[nodiscard("Ignored failure here will usually lead to an invalid application state")]]
bool command_and_confirm(
	Actuator& motor, 
	uint16_t command_register_address, uint16_t command_register_value, 
	uint16_t confirm_register_address, std::function<bool(uint16_t)> success_function, 
	const int max_wait_time_ms = 125);

}