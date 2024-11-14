#include "orca_stream.h"
#include "../actuator.h"
#include "../orca600_api/orca600_memory_map.h"

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

void OrcaStream::disconnect() {
	//reset states
	connection_state = ConnectionStatus::disconnected;
	cur_consec_failed_msgs = 0;
	modbus_client.adjust_baud_rate(UART_BAUD_RATE);
	modbus_client.adjust_response_timeout(DEFAULT_RESPONSE_uS);
}

bool OrcaStream::is_connected() {
	return connection_state == ConnectionStatus::connected;
}

void OrcaStream::initiate_handshake()
{
	if (modbus_client.get_queue_size() == 0 && has_pause_timer_expired()) {
		is_paused = false;
		num_discovery_pings_received = 0;
		enqueue_ping_msg(modbus_server_address);

		connection_state = ConnectionStatus::discovery;
	}
}

void OrcaStream::modbus_handshake(Transaction response) {

	switch (connection_state) {
	case ConnectionStatus::disconnected:
		break;
	case ConnectionStatus::discovery:
		if (response.is_echo_response() && response.is_reception_valid()) {

			num_discovery_pings_received++;

			if (num_discovery_pings_received >= connection_config.req_num_discovery_pings) {
				connection_state = ConnectionStatus::negotiation;
				enqueue_change_connection_status_fn(
					modbus_server_address,
					true,
					connection_config.target_baud_rate_bps,
					connection_config.target_delay_us);
			}
			else {
				enqueue_ping_msg(modbus_server_address);
			}
		}
		// new response was failed, or the wrong kind of response
		else {
			disconnect();
			start_pause_timer();
		}
		break;


	case ConnectionStatus::synchronization:

		if (!response.is_reception_valid()) {
			// this allows queued data to timeout or be received before reattempting a connection
			disconnect();
		}
		else {

			if (modbus_client.get_queue_size() == 0) {
				connection_state = ConnectionStatus::negotiation;
			}
		}
		break;


	case ConnectionStatus::negotiation:

		// Server responded to our change connection request with its realized baud and delay
		if (response.get_rx_function_code() == change_connection_status && response.is_reception_valid()) {
			uint8_t* rx_data = response.get_rx_data();
			modbus_client.adjust_baud_rate(
				(uint32_t(rx_data[2]) << 24)
				| (uint32_t(rx_data[3]) << 16)
				| (uint32_t(rx_data[4]) << 8)
				| (uint32_t(rx_data[5]) << 0)); //set baud

			modbus_client.adjust_interframe_delay_us(

				(uint16_t(rx_data[6]) << 8) | rx_data[7]); //set delay

			// Reduce timeouts
			modbus_client.adjust_response_timeout(connection_config.response_timeout_us);

			connection_state = ConnectionStatus::connected;

		}
		// Server failed to respond to our change connection request
		else {
			disconnect();
		}
	case ConnectionStatus::connected:
		// connection successful
		break;
	}
}

void OrcaStream::update_stream_state(Transaction response)
{
	if (!response.is_reception_valid()) {
		cur_consec_failed_msgs++;
		failed_msg_counter++;
		if (connection_state == ConnectionStatus::connected && cur_consec_failed_msgs >= connection_config.max_consec_failed_msgs) {
			disconnect();
		}
	}
	// Response was valid
	else {
		cur_consec_failed_msgs = 0;
		success_msg_counter++;
	}
}

void OrcaStream::handle_stream()
{
	// This object can queue messages on the UART with the either the handshake or the connected run loop
	if (is_enabled()) {
		if (connection_state == ConnectionStatus::connected) {
			enqueue_motor_frame();
		}
		else if (connection_state == ConnectionStatus::disconnected) {
			initiate_handshake();
		}
	}
}

void OrcaStream::set_connection_config(ConnectionConfig config) {
	connection_config = config;
}

void OrcaStream::enqueue_motor_frame() {
	if (modbus_client.get_queue_size() > 0) return;
	switch (stream_mode) {
	case MotorCommand:
		motor_stream_command();
		break;
	case MotorRead:
		motor_stream_read();
		break;
	case MotorWrite:
		motor_stream_write();
		break;
	}
}

void OrcaStream::set_stream_mode(OrcaStream::StreamMode mode) {
	stream_mode = mode;
}

void OrcaStream::motor_stream_command() {
	switch (comms_mode) {
	case ForceMode: {
		if ((modbus_client.get_system_cycles() - stream_timeout_start) > stream_timeout_cycles) {		//return to sleep mode if stream timed out
			comms_mode = SleepMode;
		}
		else {
			motor_command_fn(modbus_server_address, FORCE_CMD, force_command);
		}
		break;
	}
	case PositionMode:
		if ((modbus_client.get_system_cycles() - stream_timeout_start) > stream_timeout_cycles) {   //return to sleep mode if stream timed out
			comms_mode = SleepMode;
		}
		else {
			motor_command_fn(modbus_server_address, POS_CMD, position_command);
		}
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

void OrcaStream::motor_stream_read() {
	motor_read_fn(modbus_server_address, motor_read_width, motor_read_addr);
}

void OrcaStream::motor_stream_write() {
	motor_write_fn(modbus_server_address, motor_write_width, motor_write_addr, motor_write_data);
}

int OrcaStream::motor_command_fn(uint8_t device_address, uint8_t command_code, int32_t register_value) {
	uint8_t data_bytes[5] = {
		uint8_t(command_code),
		uint8_t(register_value >> 24),
		uint8_t(register_value >> 16),
		uint8_t(register_value >> 8),
		uint8_t(register_value)
	};
	Transaction my_temp_transaction;
	my_temp_transaction.load_transmission_data(device_address, motor_command, data_bytes, 5, get_app_reception_length(motor_command));
	int check = modbus_client.enqueue_transaction(my_temp_transaction);
	return check;
}

int OrcaStream::motor_read_fn(uint8_t device_address, uint8_t width, uint16_t register_address) {
	uint8_t data_bytes[3] = {
		uint8_t(register_address >> 8),
		uint8_t(register_address),
		uint8_t(width)
	};

	Transaction my_temp_transaction;
	my_temp_transaction.load_transmission_data(device_address, motor_read, data_bytes, 3, get_app_reception_length(motor_read));
	int check = modbus_client.enqueue_transaction(my_temp_transaction);
	return check;
}

int OrcaStream::motor_write_fn(uint8_t device_address, uint8_t width, uint16_t register_address, uint32_t register_value) {
	uint8_t data_bytes[7] = {
		uint8_t(register_address >> 8),
		uint8_t(register_address),
		uint8_t(width),
		uint8_t(register_value >> 24),
		uint8_t(register_value >> 16),
		uint8_t(register_value >> 8),
		uint8_t(register_value)
	};

	Transaction my_temp_transaction;
	my_temp_transaction.load_transmission_data(device_address, motor_write, data_bytes, 7, get_app_reception_length(motor_write));
	int check = modbus_client.enqueue_transaction(my_temp_transaction);
	return check;
}

int OrcaStream::enqueue_change_connection_status_fn(uint8_t device_address, bool connect, uint32_t baud_rate_bps, uint16_t delay_us) {
	Transaction transaction;
	uint16_t requested_state = connect ? 0xFF00 : 0;

	uint8_t data[8] = { uint8_t(requested_state >> 8),
		uint8_t(requested_state),
		uint8_t(baud_rate_bps >> 24),
		uint8_t(baud_rate_bps >> 16),
		uint8_t(baud_rate_bps >> 8),
		uint8_t(baud_rate_bps),
		uint8_t(delay_us >> 8),
		uint8_t(delay_us) };

	transaction.load_transmission_data(device_address, change_connection_status, data, 8, get_app_reception_length(change_connection_status));
	int check = modbus_client.enqueue_transaction(transaction);
	return check;
}

int OrcaStream::get_app_reception_length(uint8_t fn_code) {
	switch (fn_code) {
	case motor_command:
		return 19;
	case motor_read:
		return 24;
	case motor_write:
		return 20;
	case change_connection_status:
		return 12;
	default:
		return -1;
	}
}

void OrcaStream::update_stream_mode(MotorMode mode)
{
	comms_mode = mode;
}

void OrcaStream::update_write_stream(uint8_t width, uint16_t register_address, uint32_t register_value) {
	motor_write_data = register_value;
	motor_write_addr = register_address;
	motor_write_width = width;
}

void OrcaStream::update_read_stream(uint8_t width, uint16_t register_address) {
	motor_read_addr = register_address;
	motor_read_width = width;
}

void OrcaStream::set_force_mN(int32_t force) {
	force_command = force;
	stream_timeout_start = modbus_client.get_system_cycles();
}

void OrcaStream::set_position_um(int32_t position) {
	position_command = position;
	stream_timeout_start = modbus_client.get_system_cycles();
}

void OrcaStream::set_haptic_effects(uint16_t effects) {
	haptic_command_effects = effects;
	stream_timeout_start = modbus_client.get_system_cycles();
}

void OrcaStream::set_stream_timeout(int64_t timeout_us) {
	stream_timeout_cycles = timeout_us;
}

void OrcaStream::start_pause_timer() {
	pause_timer_start = modbus_client.get_system_cycles();
	is_paused = 1;
}

bool OrcaStream::has_pause_timer_expired() {
	if (!is_paused || (modbus_client.get_system_cycles() - pause_timer_start) >= pause_time_cycles) {//(pause_time_us*CYCLES_PER_MICROSECOND)){
		return 1;
	}
	return 0;
}

int OrcaStream::enqueue_ping_msg(uint8_t device_address) {
	Transaction ping_transaction = DefaultModbusFunctions::return_query_data_fn(device_address); //pass in num_data as 0 so nothing from data array will be loaded into transmission
	return modbus_client.enqueue_transaction(ping_transaction);
}