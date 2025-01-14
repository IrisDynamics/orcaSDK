#include "../actuator.h"

[[nodiscard("Ignored failure here will usually lead to an invalid application state")]]
bool command_and_confirm(
	Actuator& motor, 
	uint16_t command_register_address, uint16_t command_register_value, 
	uint16_t confirm_register_address, uint16_t confirm_register_value, 
	const int max_wait_time_ms = 125);
[[nodiscard("Ignored failure here will usually lead to an invalid application state")]]
bool command_and_confirm(
	Actuator& motor, 
	uint16_t command_register_address, uint16_t command_register_value, 
	uint16_t confirm_register_address, std::function<bool(uint16_t)> success_function, 
	const int max_wait_time_ms = 125);