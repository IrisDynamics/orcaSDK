#include "command_and_confirm.h"
#include <chrono>

bool orcaSDK::command_and_confirm(Actuator& motor, uint16_t command_register_address, uint16_t command_register_value, uint16_t confirm_register_address, uint16_t confirm_register_value, const int max_wait_time_ms)
{
	return command_and_confirm(
		motor,
		command_register_address,
		command_register_value,
		confirm_register_address,
		[confirm_register_value](uint16_t read_value)->bool {
			return (read_value == confirm_register_value);
		},
		max_wait_time_ms
	);

}

bool orcaSDK::command_and_confirm(Actuator& motor,
	uint16_t command_register_address,
	uint16_t command_register_value,
	uint16_t confirm_register_address,
	std::function<bool(uint16_t)> success_function,
	const int wait_time_ms)
{
	bool command_was_successful = false;

	auto start_time = std::chrono::system_clock::now();

	constexpr int reads_per_write = 3;
	int iteration_num = 0;

	while (std::chrono::system_clock::now() - start_time < std::chrono::milliseconds(wait_time_ms))
	{
		if (iteration_num % reads_per_write == 0)
		{
			motor.write_register_blocking(command_register_address, command_register_value);
		}
		else
		{
			OrcaResult<uint16_t> result = motor.read_register_blocking(confirm_register_address);
			if (!result.error && success_function(result.value))
			{
				command_was_successful = true;
				break;
			}
		}
		iteration_num++;
	}
	return command_was_successful;
}
