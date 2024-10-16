#include "../actuator.h"

#if defined(WINDOWS)
void Actuator::set_new_comport(int _comport) {
	std::shared_ptr<windows_SerialInterface> win_modbus_client = std::dynamic_pointer_cast<windows_SerialInterface>(serial_interface);
	win_modbus_client->set_new_comport(_comport);
}

void Actuator::disable_comport() {
	std::shared_ptr<windows_SerialInterface> win_modbus_client = std::dynamic_pointer_cast<windows_SerialInterface>(serial_interface);
	win_modbus_client->disable_comport_comms();
}
#endif

//Constructor
Actuator::Actuator(
	int uart_channel,
	const char* name
) :
	Actuator(
#if defined(WINDOWS)
		std::make_shared<windows_SerialInterface>(uart_channel),
#endif
		uart_channel,
		name
	)
{}

Actuator::Actuator(
	std::shared_ptr<SerialInterface> serial_interface,
	int uart_channel,
	const char* name
) :
	serial_interface(serial_interface),
	modbus_client(*serial_interface, uart_channel)
{}

void Actuator::set_mode(MotorMode orca_mode) {
	write_register(CTRL_REG_3, (uint8_t)orca_mode);
	comms_mode = orca_mode;
}

Actuator::MotorMode Actuator::get_mode() {
	return comms_mode;
}

void Actuator::set_stream_mode(StreamMode mode) {
	stream_mode = mode;
}

Actuator::StreamMode Actuator::get_stream_mode() {
	return stream_mode;
}

void Actuator::update_write_stream(uint8_t width, uint16_t register_address, uint32_t register_value) {
	motor_write_data = register_value;
	motor_write_addr = register_address;
	motor_write_width = width;
}

void Actuator::update_read_stream(uint8_t width, uint16_t register_address) {
	motor_read_addr = register_address;
	motor_read_width = width;
}

void Actuator::set_position_um(int32_t position) {
	position_command = position;
	stream_timeout_start = modbus_client.get_system_cycles();
}

int32_t Actuator::get_force_mN() {
	return uint32_t(orca_reg_contents[FORCE_REG_H_OFFSET] << 16) | orca_reg_contents[FORCE_REG_OFFSET];
}

int32_t Actuator::get_position_um() {
	return (orca_reg_contents[POS_REG_H_OFFSET] << 16) | orca_reg_contents[POS_REG_OFFSET];
}

void Actuator::enable_haptic_effects(uint16_t effects) {
	write_register(HAPTIC_STATUS, effects);
}

void Actuator::set_stream_timeout(uint64_t timeout_us) {
	stream_timeout_cycles = timeout_us;
}

void Actuator::init() {
	disconnect();	// dc is expected to return us to a good init state
	modbus_client.init(UART_BAUD_RATE);
}

void Actuator::run()
{
	run_out();
	run_in();
}

void Actuator::flush()
{
	bool current_paused_state = stream_paused;
	set_stream_paused(true);

	while (modbus_client.get_queue_size() > 0)
	{
		run();
	}

	set_stream_paused(current_paused_state);
}

void Actuator::run_out() {
	handle_stream();
	// This function results in the UART sending any data that has been queued
	modbus_client.run_out();
}

void Actuator::run_in() {


	modbus_client.run_in();

	if (modbus_client.is_response_ready()) {
		Transaction response = modbus_client.dequeue_transaction();

		if (enabled && !stream_paused && !is_connected()) modbus_handshake(response);

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

			handle_transaction_response(response);
		}
	}
}

void Actuator::handle_transaction_response(Transaction response)
{
	switch (response.get_rx_function_code()) {

	case ModbusFunctionCodes::read_holding_registers:
	case ModbusFunctionCodes::read_write_multiple_registers: {
		// add the received data to the local copy of the memory map
		u16 register_start_address = (response.get_tx_data()[0] << 8) + response.get_tx_data()[1];
		u16 num_registers = (response.get_tx_data()[2] << 8) + response.get_tx_data()[3];
		for (int i = 0; i < num_registers; i++) {
			u16 register_data = (response.get_rx_data()[1 + i * 2] << 8) + response.get_rx_data()[2 + i * 2];
			orca_reg_contents[register_start_address + i] = register_data;
		}
		break;
	}
	case ModbusFunctionCodes::write_single_register:
		// nothing to do
		break;

	case motor_command:
		orca_reg_contents[POS_REG_H_OFFSET] = (response.get_rx_data()[0] << 8) | response.get_rx_data()[1];
		orca_reg_contents[POS_REG_OFFSET] = (response.get_rx_data()[2] << 8) | response.get_rx_data()[3];
		orca_reg_contents[FORCE_REG_H_OFFSET] = (response.get_rx_data()[4] << 8) | response.get_rx_data()[5];
		orca_reg_contents[FORCE_REG_OFFSET] = (response.get_rx_data()[6] << 8) | response.get_rx_data()[7];
		orca_reg_contents[POWER_REG_OFFSET] = (response.get_rx_data()[8] << 8) | response.get_rx_data()[9];
		orca_reg_contents[TEMP_REG_OFFSET] = (response.get_rx_data()[10]);
		orca_reg_contents[VOLTAGE_REG_OFFSET] = (response.get_rx_data()[11] << 8) | response.get_rx_data()[12];
		orca_reg_contents[ERROR_REG_OFFSET] = (response.get_rx_data()[13] << 8) | response.get_rx_data()[14];
		break;

	case motor_read: {
		u16 register_start_address = (response.get_tx_data()[0] << 8) + response.get_tx_data()[1];
		u8 width = response.get_tx_data()[2];
		u16 register_data = (response.get_rx_data()[2] << 8) + response.get_rx_data()[3];
		orca_reg_contents[register_start_address] = register_data;
		if (width > 1) {
			register_data = (response.get_rx_data()[0] << 8) + response.get_rx_data()[1];
			orca_reg_contents[register_start_address + 1] = register_data;
		}
		orca_reg_contents[MODE_OF_OPERATION] = response.get_rx_data()[4];
		orca_reg_contents[POS_REG_H_OFFSET] = (response.get_rx_data()[5] << 8) | response.get_rx_data()[6];
		orca_reg_contents[POS_REG_OFFSET] = (response.get_rx_data()[7] << 8) | response.get_rx_data()[8];
		orca_reg_contents[FORCE_REG_H_OFFSET] = (response.get_rx_data()[9] << 8) | response.get_rx_data()[10];
		orca_reg_contents[FORCE_REG_OFFSET] = (response.get_rx_data()[11] << 8) | response.get_rx_data()[12];
		orca_reg_contents[POWER_REG_OFFSET] = (response.get_rx_data()[13] << 8) | response.get_rx_data()[14];
		orca_reg_contents[TEMP_REG_OFFSET] = (response.get_rx_data()[15]);
		orca_reg_contents[VOLTAGE_REG_OFFSET] = (response.get_rx_data()[16] << 8) | response.get_rx_data()[17];
		orca_reg_contents[ERROR_REG_OFFSET] = (response.get_rx_data()[18] << 8) | response.get_rx_data()[19];
	}
				   break;
	case motor_write:
		orca_reg_contents[MODE_OF_OPERATION] = response.get_rx_data()[0];
		orca_reg_contents[POS_REG_H_OFFSET] = (response.get_rx_data()[1] << 8) | response.get_rx_data()[2];
		orca_reg_contents[POS_REG_OFFSET] = (response.get_rx_data()[3] << 8) | response.get_rx_data()[4];
		orca_reg_contents[FORCE_REG_H_OFFSET] = (response.get_rx_data()[5] << 8) | response.get_rx_data()[6];
		orca_reg_contents[FORCE_REG_OFFSET] = (response.get_rx_data()[7] << 8) | response.get_rx_data()[8];
		orca_reg_contents[POWER_REG_OFFSET] = (response.get_rx_data()[9] << 8) | response.get_rx_data()[10];
		orca_reg_contents[TEMP_REG_OFFSET] = (response.get_rx_data()[11]);
		orca_reg_contents[VOLTAGE_REG_OFFSET] = (response.get_rx_data()[12] << 8) | response.get_rx_data()[13];
		orca_reg_contents[ERROR_REG_OFFSET] = (response.get_rx_data()[14] << 8) | response.get_rx_data()[15];
		break;

	case ModbusFunctionCodes::read_coils:
	case ModbusFunctionCodes::read_discrete_inputs:
	case ModbusFunctionCodes::read_input_registers:
	case ModbusFunctionCodes::write_single_coil:
	case ModbusFunctionCodes::read_exception_status:
	case ModbusFunctionCodes::diagnostics:
	case ModbusFunctionCodes::get_comm_event_counter:
	case ModbusFunctionCodes::get_comm_event_log:
	case ModbusFunctionCodes::write_multiple_coils:
	case ModbusFunctionCodes::write_multiple_registers:
	case ModbusFunctionCodes::report_server_id:
	case ModbusFunctionCodes::mask_write_register:
	default:
		// todo: warn about un-implemented function codes being received
		break;
	}
}

const char* Actuator::get_name() {
	return my_name;
}

int Actuator::channel_number() {
	return modbus_client.channel_number;
}

uint16_t Actuator::get_mode_of_operation() {
	return orca_reg_contents[MODE_OF_OPERATION];
}

uint16_t Actuator::get_power_W() {
	return orca_reg_contents[POWER_REG_OFFSET];
}

uint8_t Actuator::get_temperature_C() {
	return orca_reg_contents[TEMP_REG_OFFSET];
}

uint16_t Actuator::get_voltage_mV() {
	return orca_reg_contents[VOLTAGE_REG_OFFSET];
}

uint16_t Actuator::get_errors() {
	return orca_reg_contents[ERROR_REG_OFFSET];
}

uint32_t Actuator::get_serial_number() {

	uint16_t sn_high = orca_reg_contents[SERIAL_NUMBER_HIGH];
	uint16_t sn_low = orca_reg_contents[SERIAL_NUMBER_LOW];

	uint32_t serial_number = (sn_high * (1 << 16)) + sn_low;
	return serial_number;
}

uint16_t Actuator::get_major_version() {
	return orca_reg_contents[MAJOR_VERSION];
}

uint16_t Actuator::get_release_state() {
	return orca_reg_contents[RELEASE_STATE];
}

uint16_t Actuator::get_revision_number() {
	return orca_reg_contents[REVISION_NUMBER];
}

bool Actuator::version_is_at_least(uint8_t version, uint8_t release_state, uint8_t revision_number) {
	return
		get_major_version() > version
		|| (get_major_version() == version && get_revision_number() > revision_number)
		|| (get_major_version() == version && get_revision_number() == revision_number && get_release_state() >= release_state);
}

void Actuator::zero_position() {
	write_register(ZERO_POS_REG_OFFSET, ZERO_POS_MASK);
}

void Actuator::clear_errors() {
	write_register(CLEAR_ERROR_REG_OFFSET, CLEAR_ERROR_MASK);
}

void Actuator::get_latched_errors() {
	read_register(ERROR_1);
}

void Actuator::set_max_force(s32 max_force) {
	uint8_t data[4] = {
		uint8_t(max_force >> 8),
		uint8_t(max_force),
		uint8_t(max_force >> 24),
		uint8_t(max_force >> 16)
	};
	write_registers(USER_MAX_FORCE, 2, data);
}

void Actuator::set_max_temp(uint16_t max_temp) {
	write_register(USER_MAX_TEMP, max_temp);
}

void Actuator::set_max_power(uint16_t max_power) {
	write_register(USER_MAX_POWER, max_power);
}

void Actuator::set_pctrl_tune_softstart(uint16_t t_in_ms) {
	write_register(PC_SOFTSTART_PERIOD, t_in_ms);
}

void Actuator::set_safety_damping(uint16_t max_safety_damping) {
	write_register(SAFETY_DGAIN, max_safety_damping);
}

void Actuator::tune_position_controller(uint16_t pgain, uint16_t igain, uint16_t dvgain, uint32_t sat, uint16_t degain) {

	uint8_t data[12] = {
		uint8_t(pgain >> 8),
		uint8_t(pgain),
		uint8_t(igain >> 8),
		uint8_t(igain),
		uint8_t(dvgain >> 8),
		uint8_t(dvgain),
		uint8_t(degain >> 8),
		uint8_t(degain),
		uint8_t(sat >> 8),
		uint8_t(sat),
		uint8_t(sat >> 24),
		uint8_t(sat >> 16)
	};

	write_registers(PC_PGAIN, 6, data);
	write_register(CONTROL_REG_1::address, CONTROL_REG_1::position_controller_gain_set_flag);
}

void Actuator::set_kinematic_motion(int ID, int32_t position, int32_t time, int16_t delay, int8_t type, int8_t auto_next, int8_t next_id) {
	if (next_id == -1) {
		next_id = ID + 1;
	}

	uint8_t data[12] = {
		uint8_t(position >> 8),
		uint8_t(position),
		uint8_t(position >> 24),
		uint8_t(position >> 16),
		uint8_t(time >> 8),
		uint8_t(time),
		uint8_t(time >> 24),
		uint8_t(time >> 16),
		uint8_t(delay >> 8),
		uint8_t(delay),
		uint8_t(0),
		uint8_t((type << 1) | (next_id << 3) | auto_next)
	};
	write_registers(KIN_MOTION_0 + (6 * ID), 6, data);
}

void Actuator::set_spring_effect(u8 spring_id, u16 gain, u32 center, u16 dead_zone, u16 saturation, u8 coupling) {
	u8 data[12] = {
		u8(gain >> 8),
		u8(gain),
		u8(center >> 8),
		u8(center),
		u8(center >> 24),
		u8(center >> 16),
		u8(0),
		u8(coupling),
		u8(dead_zone >> 8),
		u8(dead_zone),
		u8(saturation >> 8),
		u8(saturation),

	};
	write_registers(S0_GAIN_N_MM + spring_id * 6, 6, data);
}

void Actuator::set_osc_effect(u8 osc_id, u16 amplitude, u16 frequency_dhz, u16 duty, u16 type) {
	u8 data[8] = {
		u8(amplitude >> 8),
		u8(amplitude),
		u8(type >> 8),
		u8(type),
		u8(frequency_dhz >> 8),
		u8(frequency_dhz),
		u8(duty >> 8),
		u8(duty)
	};
	write_registers(O0_GAIN_N + osc_id * 4, 4, data);
}

void Actuator::trigger_kinematic_motion(int ID) {
	write_register(KIN_SW_TRIGGER, ID);
}

void Actuator::read_register(uint16_t reg_address, MessagePriority priority) {
	modbus_client.enqueue_transaction(DefaultModbusFunctions::read_holding_registers_fn(connection_config.server_address, reg_address, 1, priority));
}

void Actuator::read_registers(uint16_t reg_address, uint16_t num_registers, MessagePriority priority) {
	if (num_registers < 1 || num_registers > MAX_NUM_READ_REG) return;

	modbus_client.enqueue_transaction(DefaultModbusFunctions::read_holding_registers_fn(connection_config.server_address, reg_address, num_registers, priority));
}

void Actuator::write_register(uint16_t reg_address, uint16_t reg_data, MessagePriority priority) {
	modbus_client.enqueue_transaction(DefaultModbusFunctions::write_single_register_fn(connection_config.server_address, reg_address, reg_data, priority));
}

void Actuator::write_registers(uint16_t reg_address, uint16_t num_registers, uint8_t* reg_data, MessagePriority priority) {
	if (num_registers < 1 || num_registers > MAX_NUM_WRITE_REG) return;

	modbus_client.enqueue_transaction(DefaultModbusFunctions::write_multiple_registers_fn(connection_config.server_address, reg_address, num_registers, reg_data, priority));
}

void Actuator::write_registers(uint16_t reg_address, uint16_t num_registers, uint16_t* reg_data, MessagePriority priority) {
	if (num_registers < 1 || num_registers > MAX_NUM_WRITE_REG) return;

	uint8_t data[126];
	for (int i = 0; i < num_registers; i++) {
		data[i * 2] = reg_data[i] >> 8;
		data[i * 2 + 1] = reg_data[i];
	}
	modbus_client.enqueue_transaction(DefaultModbusFunctions::write_multiple_registers_fn(connection_config.server_address, reg_address, num_registers, data, priority));
}

void Actuator::read_write_registers(uint16_t read_starting_address, uint16_t read_num_registers, uint16_t write_starting_address, uint16_t write_num_registers, uint8_t* write_data, MessagePriority priority)
{
	if (read_num_registers < 1 || read_num_registers > MAX_NUM_READ_REG) return;
	if (write_num_registers < 1 || write_num_registers > MAX_NUM_WRITE_REG) return;

	modbus_client.enqueue_transaction(DefaultModbusFunctions::read_write_multiple_registers_fn(
		connection_config.server_address,
		read_starting_address, read_num_registers,
		write_starting_address, write_num_registers,
		write_data,
		priority
	));
}

uint16_t Actuator::get_orca_reg_content(uint16_t offset) {
	return orca_reg_contents[offset];
}

void Actuator::begin_serial_logging()
{
#ifdef WINDOWS
	std::shared_ptr<Log> app_log = std::make_shared<Log>();
	app_log->set_verbose_mode(false);
	app_log->open(my_name);
#endif
	begin_serial_logging(app_log);
}

void Actuator::begin_serial_logging(std::shared_ptr<LogInterface> log)
{
	modbus_client.begin_logging(log);
}

void Actuator::set_stream_paused(bool paused)
{
	stream_paused = paused;
}

void Actuator::synchronize_memory_map() {
	read_registers(PARAM_REG_START, PARAM_REG_SIZE);
	read_registers(ERROR_0, ADC_DATA_COLLISION - ERROR_0);
	//read_registers(STATOR_CAL_REG_START	, STATOR_CAL_REG_SIZE			) ;
	//read_registers(SHAFT_CAL_REG_START 	, SHAFT_CAL_REG_SIZE 			) ;
	//read_registers(FORCE_CAL_REG_START 	, FORCE_CAL_REG_SIZE 			) ;
	read_registers(TUNING_REG_START, TUNING_REG_SIZE);
}

void Actuator::desynchronize_memory_map() {
	for (int i = 0; i < ORCA_REG_SIZE; i++) {
		orca_reg_contents[i] = 0;
	}
}

void Actuator::motor_stream_command() {
	switch (comms_mode) {
		;
	case ForceMode: {
		if (uint32_t(modbus_client.get_system_cycles() - stream_timeout_start) > stream_timeout_cycles) {		//return to sleep mode if stream timed out
			comms_mode = SleepMode;
		}
		else {
			motor_command_fn(connection_config.server_address, FORCE_CMD, force_command);
		}
		break;
	}
	case PositionMode:
		if (uint32_t(modbus_client.get_system_cycles() - stream_timeout_start) > stream_timeout_cycles) {   //return to sleep mode if stream timed out
			comms_mode = SleepMode;
		}
		else {
			motor_command_fn(connection_config.server_address, POS_CMD, position_command);
		}
		break;
	case KinematicMode:
		motor_command_fn(connection_config.server_address, kinematic_command, 0);
		break;
	case HapticMode:
		motor_command_fn(connection_config.server_address, haptic_command, 0);
		break;
	default:
		motor_command_fn(connection_config.server_address, 0, 0); //any register address other than force or position register_adresses will induce sleep mode and provided register_value will be ignored
		break;
	}
}

void Actuator::motor_stream_read() {
	motor_read_fn(connection_config.server_address, motor_read_width, motor_read_addr);
}

void Actuator::motor_stream_write() {
	motor_write_fn(connection_config.server_address, motor_write_width, motor_write_addr, motor_write_data);
}

void Actuator::enqueue_motor_frame() {
	if (modbus_client.get_queue_size() >= 2) return;
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

int Actuator::get_app_reception_length(uint8_t fn_code) {
	switch (fn_code) {
	case motor_command:
		return 19;
	case motor_read:
		return 24;
	case motor_write:
		return 20;
	case Actuator::change_connection_status:
		return 12;
	default:
		return -1;
	}
}

int Actuator::motor_command_fn(uint8_t device_address, uint8_t command_code, int32_t register_value) {
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

int Actuator::motor_read_fn(uint8_t device_address, uint8_t width, uint16_t register_address) {
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

int Actuator::motor_write_fn(uint8_t device_address, uint8_t width, uint16_t register_address, uint32_t register_value) {
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

int Actuator::set_connection_config(Actuator::ConnectionConfig config) {

	if (config.server_address < 1 || config.server_address > 247) {
		return 0;   //todo : standardize error value
	}

	connection_config = config;

	return 1;   //todo : standardize error value
}

void Actuator::handle_stream()
{
	// This object can queue messages on the UART with the either the handshake or the connected run loop
	if (is_enabled() && !stream_paused) {
		if (connection_state == ConnectionStatus::connected) {
			enqueue_motor_frame();
		}
		else if (connection_state == ConnectionStatus::disconnected) {
			initiate_handshake();
		}
	}
}

bool Actuator::is_enabled() {
	return enabled;
}

void Actuator::enable() {
	enabled = true;
}

void Actuator::disable() {
	enabled = false;
	if (is_connected()) {
		enqueue_change_connection_status_fn(connection_config.server_address, false, 0, 0);
	}
	disconnect();
}

bool Actuator::is_connected() {
	return connection_state == ConnectionStatus::connected;
}

void Actuator::disconnect() {
	//reset states
	connection_state = ConnectionStatus::disconnected;
	cur_consec_failed_msgs = 0;
	modbus_client.adjust_baud_rate(UART_BAUD_RATE);
	modbus_client.adjust_response_timeout(DEFAULT_RESPONSE_uS);
	//is_paused = true;// pause to allow server to reset to disconnected state

	//start_pause_timer();
	desynchronize_memory_map();
}

void Actuator::initiate_handshake()
{
	if (modbus_client.get_queue_size() == 0 && has_pause_timer_expired()) {
		is_paused = false;
		num_discovery_pings_received = 0;
		enqueue_ping_msg();

		connection_state = ConnectionStatus::discovery;
	}
}

void Actuator::modbus_handshake(Transaction response) {

	switch (connection_state) {
	case ConnectionStatus::disconnected:
		break;
	case ConnectionStatus::discovery:
		if (response.is_echo_response() && response.is_reception_valid()) {

			num_discovery_pings_received++;

			if (num_discovery_pings_received >= connection_config.req_num_discovery_pings) {
				synchronize_memory_map();			// loads many read messages onto the queue
				connection_state = ConnectionStatus::synchronization;

			}
			else {
				enqueue_ping_msg();
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
				enqueue_change_connection_status_fn(
					connection_config.server_address,
					true,
					connection_config.target_baud_rate_bps,
					connection_config.target_delay_us);
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

int Actuator::enqueue_change_connection_status_fn(uint8_t device_address, bool connect, uint32_t baud_rate_bps, uint16_t delay_us) {
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

void Actuator::start_pause_timer() {
	pause_timer_start = modbus_client.get_system_cycles();
	is_paused = 1;
}

bool Actuator::has_pause_timer_expired() {
	if (!is_paused || (u32)(modbus_client.get_system_cycles() - pause_timer_start) >= pause_time_cycles) {//(pause_time_us*CYCLES_PER_MICROSECOND)){
		return 1;
	}
	return 0;
}

int Actuator::enqueue_ping_msg() {
	constexpr Transaction ping_transaction = DefaultModbusFunctions::return_query_data_fn(1/*connection_config.server_address*/); //pass in num_data as 0 so nothing from data array will be loaded into transmission
	return modbus_client.enqueue_transaction(ping_transaction);
}
