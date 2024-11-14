#include "../actuator.h"


bool command_and_confirm(Actuator& motor, uint16_t command_register_address, uint16_t command_register_value, uint16_t confirm_register_address, uint16_t confirm_register_value, const int num_command_confirm_retries = 25);
bool command_and_confirm(Actuator& motor, uint16_t command_register_address, uint16_t command_register_value, uint16_t confirm_register_address, std::function<bool(uint16_t)> success_function, const int num_command_confirm_retries = 25);


/**
 *	@overload	bool Actuator::command_and_confirm(uint16_t command_register_address, uint16_t command_register_value, uint16_t confirm_register_address, uint16_t confirm_register_value);
	*	@brief	Writes to a register and blocks the current thread until a read register matches a given value.
	*	@param	confirm_register_value	The value that the register in confirm_register_address should have
	*									for the command to have been considered a success
	*/
[[nodiscard("Ignored failure here will usually lead to an invalid application state")]]
bool command_and_confirm(Actuator& motor, uint16_t command_register_address, uint16_t command_register_value, uint16_t confirm_register_address, uint16_t confirm_register_value, const int num_command_confirm_retries)
{
	return command_and_confirm(
		motor,
		command_register_address,
		command_register_value,
		confirm_register_address,
		[confirm_register_value](uint16_t read_value)->bool {
			return (read_value == confirm_register_value); 
		},
		num_command_confirm_retries
		);

}

/**
 *	@brief	Writes to a register and blocks the current thread until some post-condition is observed.
	*	@details	Writes to modbus address <command_register_address> with value
	*				<command_register_value> while reading from <confirm_register_address>. Will
	*				repeatedly perform this write and read while calling <success_function> until
	*				it returns a value of true.
	*	@param	command_register_address	The register being written to
	*	@param	command_register_value	The value to be written
	*	@param	confirm_register_address	The register that should be read from for confirmation
	*	@param	success_function	The function that must return true for the command to have been considered a success
	*/
[[nodiscard("Ignored failure here will usually lead to an invalid application state")]]
bool command_and_confirm(Actuator& motor, uint16_t command_register_address, uint16_t command_register_value, uint16_t confirm_register_address, std::function<bool(uint16_t)> success_function, const int num_command_confirm_retries)
{

	static constexpr int num_reads_per_command_retries = 3;

	bool command_was_successful = false;
	for (int i = 0; i < num_command_confirm_retries; i++)
	{
		if ((i % num_reads_per_command_retries) == 0) motor.write_register_blocking(command_register_address, command_register_value);
		auto [current_read, error] = motor.read_register_blocking(confirm_register_address);
		if (!error && success_function(current_read))
		{
			command_was_successful = true;
			break;
		}
	}
	return command_was_successful;
}
