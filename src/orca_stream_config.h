#pragma once

/**
 * @brief Configurable parameters for the handshake sequence and connection maintenance. Should be set when disabled
*/
struct ConnectionConfig {
	int req_num_discovery_pings = 3; //3      //number of sucessful comms check messages required to move to next step in handshake sequence
	int max_consec_failed_msgs = 10;      //number of failed/missed messages to trigger disconnect
	uint32_t target_baud_rate_bps = 625000;
	uint16_t target_delay_us = 80;
	uint32_t response_timeout_us = 8000;  /// this timeout will be used to override the default response timeout after a handshake succeeds and a new baud rate is negotiated.
};

/**
  @brief Enum of all actuator specific function codes, in decimal.
 */
enum orca_function_codes_e {
	motor_command = 100,
	motor_read = 104,
	motor_write = 105
};

enum connection_function_codes_e {
	change_connection_status = 65
};