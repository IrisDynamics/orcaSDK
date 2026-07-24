#pragma once

namespace orcaSDK
{

/**
  @brief Enum of all actuator specific function codes, in decimal.
 */
enum orca_function_codes_e {
	motor_command = 100,
	motor_write = 105
};

enum connection_function_codes_e {
	change_connection_status = 65
};

}