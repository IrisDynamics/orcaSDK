#pragma once

#include "modbus_client.h"
#include "orca_stream_config.h"
#include "orca_modes.h"

namespace orcaSDK
{

class Actuator;

class OrcaStream
{
public:

	OrcaStream(Actuator* motor, ModbusClient& modbus_client, uint8_t modbus_server_address);

	enum class StreamType {
		Command,
		Write,
		Read
	};

	/**
	 * @brief Determine if communication with a server is enabled or not
	 *
	 * @return boolean - the enabled status (true if enabled, false otherwise)
	*/
	bool is_enabled();

	void enable(StreamType type = StreamType::Command);

	void disable();

	void handle_stream();

	void update_motor_mode(MotorMode mode);

	void set_force_mN(int32_t force);

	void set_position_um(int32_t position);

	void set_haptic_effects(uint16_t effects);

	void set_write_value(uint16_t addr, int32_t value, uint8_t width = 1);

private:
	Actuator* motor;
	ModbusClient& modbus_client;

	const uint8_t modbus_server_address;

	bool enabled = false;

	MotorMode comms_mode = SleepMode;

	// Used to hold the last commanded force and position commands from the user of this object
	int32_t force_command = 0;
	int32_t position_command = 0;
	uint16_t haptic_command_effects = 0;
	int32_t write_value = 0;
	uint16_t write_addr = 0;
	uint8_t write_width = 1;
	StreamType active_stream_mode = StreamType::Command;


	static constexpr int kinematic_command_code = 32;
	static constexpr int haptic_command_code = 34;

	void motor_stream_command();
	void motor_stream_write();

	/**
	  @brief Format a motor command request, function code 0x64, and add the request to the buffer queue

	  @param device_address Server device address
	  @param command_code command code to specify command mode (sleep, force, position etc.)
	  @param register_value The value to write to the register
	 */
	void motor_command_fn(uint8_t device_address, uint8_t command_code, int32_t register_value);

	void motor_write_fn(uint8_t device_address, uint16_t register_addr, int32_t register_value, uint8_t width = 1);

	/**
	 * @brief Determine the length of the request for an application specific function code
	 *
	 * @param fn_code application specific function code
	 * @return int - length of request
	 */
	int get_app_reception_length(uint8_t fn_code);
};

}