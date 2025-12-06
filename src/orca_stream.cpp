#include "orca_stream.h"
#include "../actuator.h"

namespace orcaSDK
{

OrcaStream::OrcaStream(Actuator* motor, ModbusClient& modbus_client, uint8_t modbus_server_address) :
	motor(motor),
	modbus_client(modbus_client),
	modbus_server_address(modbus_server_address)
{}

bool OrcaStream::is_enabled() {
	return enabled;
}

void OrcaStream::enable() {
	enabled = true;
}

void OrcaStream::disable() {
	enabled = false;
}

void OrcaStream::handle_stream()
{
	// This object can queue messages on the UART with the either the handshake or the connected run loop
	if (is_enabled() && modbus_client.get_queue_size() == 0) {
		motor_stream_command();
	}
}

void OrcaStream::motor_stream_command() {
	switch (comms_mode) {
	case ForceMode: 
		motor_command_fn(modbus_server_address, FORCE_CMD, force_command);
		break;
	case PositionMode:
		motor_command_fn(modbus_server_address, POS_CMD, position_command);
		break;
	case KinematicMode:
		motor_command_fn(modbus_server_address, kinematic_command_code, 0);
		break;
	case HapticMode:
		motor_command_fn(modbus_server_address, haptic_command_code, haptic_command_effects);
		break;
	default:
		motor_command_fn(modbus_server_address, 0, 0); //any register address other than force or position register_adresses will induce sleep mode and provided register_value will be ignored
		break;
	}
}

void OrcaStream::motor_command_fn(uint8_t device_address, uint8_t command_code, int32_t register_value) {
	uint8_t data_bytes[5] = {
		uint8_t(command_code),
		uint8_t(register_value >> 24),
		uint8_t(register_value >> 16),
		uint8_t(register_value >> 8),
		uint8_t(register_value)
	};
	Transaction my_temp_transaction;
	my_temp_transaction.load_transmission_data(device_address, motor_command, data_bytes, 5, get_app_reception_length(motor_command));
	modbus_client.enqueue_transaction(my_temp_transaction);
}

int OrcaStream::get_app_reception_length(uint8_t fn_code) {
	switch (fn_code) {
	case motor_command:
		return 19;
	case change_connection_status:
		return 12;
	default:
		return -1;
	}
}

void OrcaStream::update_motor_mode(MotorMode mode)
{
	comms_mode = mode; 
}

void OrcaStream::set_force_mN(int32_t force) {
	force_command = force;
}

void OrcaStream::set_position_um(int32_t position) {
	position_command = position;
}

void OrcaStream::set_haptic_effects(uint16_t effects) {
	haptic_command_effects = effects;
}

}