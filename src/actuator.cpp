#include "../actuator.h"
#include "chrono_clock.h"

int32_t combine_into_wide_register(uint16_t low_reg_value, uint16_t high_reg_value)
{
	return ((int32_t)high_reg_value << 16) + low_reg_value;
}

//Constructor
Actuator::Actuator(
	int serial_port_channel,
	const char* name,
	uint8_t modbus_server_address
) :
	Actuator(
#if defined(WINDOWS)
		std::make_shared<windows_SerialInterface>(serial_port_channel),
#endif
		std::make_shared<ChronoClock>(),
		serial_port_channel,
		name,
		modbus_server_address
	)
{}

Actuator::Actuator(
	std::shared_ptr<SerialInterface> serial_interface,
	std::shared_ptr<Clock> clock,
	int serial_port_channel,
	const char* name,
	uint8_t modbus_server_address
) :
	serial_interface(serial_interface),
	clock(clock),
	modbus_client(*serial_interface, *clock, serial_port_channel),
	name(name),
	stream(this, modbus_client, modbus_server_address),
	modbus_server_address(modbus_server_address)
{}

OrcaError Actuator::open_serial_port() {
	stream.disconnect();	// dc is expected to return us to a good init state
	return modbus_client.init(UART_BAUD_RATE);
}

void Actuator::set_new_serial_port(int _comport) {
	serial_interface->set_new_serial_port(_comport);
}

void Actuator::close_serial_port() {
	serial_interface->close_serial_port();
}

int Actuator::channel_number() {
	return modbus_client.channel_number;
}

OrcaError Actuator::set_mode(MotorMode orca_mode) {
	OrcaError error = write_register_blocking(CTRL_REG_3, (uint16_t)orca_mode);
	if (error) return error;
	stream.update_stream_mode(orca_mode);
	return error;
}

OrcaResult<MotorMode> Actuator::get_mode() {
	auto return_struct = read_register_blocking(MODE_OF_OPERATION);
	return {(MotorMode)return_struct.value, return_struct.error};
}

void Actuator::set_stream_mode(OrcaStream::StreamMode mode) {
	stream.set_stream_mode(mode);
}

void Actuator::update_write_stream(uint8_t width, uint16_t register_address, uint32_t register_value) {
	stream.update_write_stream(width, register_address, register_value);
}

void Actuator::update_read_stream(uint8_t width, uint16_t register_address) {
	stream.update_read_stream(width, register_address);
}

void Actuator::set_force_mN(int32_t force) {
	stream.set_force_mN(force);
}

void Actuator::set_position_um(int32_t position) {
	stream.set_position_um(position);
}

OrcaResult<int32_t> Actuator::read_wide_register_blocking(uint16_t reg_address, MessagePriority priority)
{
	modbus_client.enqueue_transaction(DefaultModbusFunctions::read_holding_registers_fn(modbus_server_address, reg_address, 2, priority));
	flush();
	if (message_error) return { 0, message_error };
	return { combine_into_wide_register(message_data[0], message_data[1]), message_error };
}

OrcaResult<uint16_t> Actuator::read_register_blocking(uint16_t reg_address, MessagePriority priority)
{
	modbus_client.enqueue_transaction(DefaultModbusFunctions::read_holding_registers_fn(modbus_server_address, reg_address, 1, priority));
	flush();
	if (message_error) return { 0, message_error };
	return { message_data[0], message_error};
}

OrcaResult<std::vector<uint16_t>> Actuator::read_multiple_registers_blocking(uint16_t reg_start_address, uint8_t num_registers, MessagePriority priority)
{
	if (num_registers == 0) return { {}, OrcaError{0} };

	modbus_client.enqueue_transaction(DefaultModbusFunctions::read_holding_registers_fn(modbus_server_address, reg_start_address, num_registers, priority));
	flush();

	return { message_data, message_error };
}

OrcaError Actuator::write_register_blocking(uint16_t reg_address, uint16_t write_data, MessagePriority priority)
{
	modbus_client.enqueue_transaction(DefaultModbusFunctions::write_single_register_fn(modbus_server_address, reg_address, write_data, priority));
	flush();
	return message_error;
}

OrcaError Actuator::write_wide_register_blocking(uint16_t reg_address, int32_t write_data, MessagePriority priority)
{
	uint16_t split_data[2]{
		uint16_t(write_data),
		uint16_t(write_data >> 16)
	};
	return write_multiple_registers_blocking(reg_address, 2, split_data, priority);
}

OrcaError Actuator::write_multiple_registers_blocking(uint16_t reg_start_address, uint8_t num_registers, uint16_t* write_data, MessagePriority priority)
{
	if (num_registers == 0) return OrcaError{ 0 };

	uint8_t data[128];
	for (int i = 0; i < num_registers; i++) {
		data[i * 2] = uint8_t(write_data[i] >> 8);
		data[i * 2 + 1] = uint8_t(write_data[i]);
	}
	modbus_client.enqueue_transaction(DefaultModbusFunctions::write_multiple_registers_fn(modbus_server_address, reg_start_address, num_registers, data, priority));
	flush();
	return message_error;
}

OrcaResult<std::vector<uint16_t>> Actuator::read_write_multiple_registers_blocking(
	uint16_t read_starting_address, uint8_t read_num_registers,
	uint16_t write_starting_address, uint8_t write_num_registers,
	uint16_t* write_data,
	MessagePriority priority)
{
	uint8_t data[128];
	for (int i = 0; i < write_num_registers; i++) {
		data[i * 2] = uint8_t(write_data[i] >> 8);
		data[i * 2 + 1] = uint8_t(write_data[i]);
	}

	modbus_client.enqueue_transaction(DefaultModbusFunctions::read_write_multiple_registers_fn(
		modbus_server_address,
		read_starting_address, read_num_registers,
		write_starting_address, write_num_registers,
		data, priority));
	flush();

	return { message_data, message_error };
}

OrcaResult<int32_t> Actuator::get_force_mN() {
	return read_wide_register_blocking(FORCE);
}

OrcaResult<int32_t> Actuator::get_position_um() {
	return read_wide_register_blocking(SHAFT_POS_UM);
}

void Actuator::update_haptic_stream_effects(uint16_t effects) {
	stream.set_haptic_effects(effects);
}

OrcaError Actuator::enable_haptic_effects(uint16_t effects) {
	return write_register_blocking(HAPTIC_STATUS, effects);
}

void Actuator::run()
{
	run_in();
	run_out();
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
	if (!stream_paused) stream.handle_stream();
	// This function results in the UART sending any data that has been queued
	modbus_client.run_out();
}

void Actuator::run_in() {
	modbus_client.run_in();

	if (modbus_client.is_response_ready()) {
		Transaction response = modbus_client.dequeue_transaction();

		if (stream.is_enabled() && !stream_paused && !stream_is_established()) stream.modbus_handshake(response);

		stream.update_stream_state(response);

		handle_transaction_response(response);
	}
}

void Actuator::handle_transaction_response(Transaction response)
{
	message_data.clear();

	int ec = response.get_failure_codes();

	std::stringstream error_message;
	if (ec & (1 << Transaction::RESPONSE_TIMEOUT_ERROR)) error_message << "\tResponse timed out. The motor took too long to respond. \n";
	if (ec & (1 << Transaction::INTERCHAR_TIMEOUT_ERROR)) error_message << "\tUnexpected interchar delay timeout. \n";
	if (ec & (1 << Transaction::UNEXPECTED_RESPONDER)) error_message << "\tWrong modbus response address. \n";
	if (ec & (1 << Transaction::CRC_ERROR)) error_message << "\tWrong CRC. \r";

	message_error = OrcaError{response.get_failure_codes(), error_message.str()};

	switch (response.get_rx_function_code()) {

	case ModbusFunctionCodes::read_holding_registers:
	case ModbusFunctionCodes::read_write_multiple_registers: {
		// add the received data to the local copy of the memory map
		//u16 register_start_address = (response.get_tx_data()[0] << 8) + response.get_tx_data()[1];
		u16 num_registers = (response.get_tx_data()[2] << 8) + response.get_tx_data()[3];
		for (int i = 0; i < num_registers; i++) {
			u16 register_data = (response.get_rx_data()[1 + i * 2] << 8) + response.get_rx_data()[2 + i * 2];
			message_data.push_back(register_data);
		}
		break;
	}
	case motor_command: {
		stream_cache.mode = 0;
		uint16_t position_high = (response.get_rx_data()[0] << 8) | response.get_rx_data()[1];
		uint16_t position_low = (response.get_rx_data()[2] << 8) | response.get_rx_data()[3];
		stream_cache.position = combine_into_wide_register(position_low, position_high);
		uint16_t force_high = (response.get_rx_data()[4] << 8) | response.get_rx_data()[5];
		uint16_t force_low = (response.get_rx_data()[6] << 8) | response.get_rx_data()[7];
		stream_cache.force = combine_into_wide_register(force_low, force_high);
		stream_cache.power = (response.get_rx_data()[8] << 8) | response.get_rx_data()[9];
		stream_cache.temperature = (response.get_rx_data()[10]);
		stream_cache.voltage = (response.get_rx_data()[11] << 8) | response.get_rx_data()[12];
		stream_cache.errors = (response.get_rx_data()[13] << 8) | response.get_rx_data()[14];
		break;
	}
	case motor_read: {
		u8 width = response.get_tx_data()[2];
		if (width == 0)
		{
			uint16_t reg_data = (response.get_rx_data()[2] << 8) + response.get_rx_data()[3];
			stream_cache.read_stream_reg = reg_data; 
		}
		else
		{
			uint16_t reg_data_low = (response.get_rx_data()[2] << 8) + response.get_rx_data()[3];
			uint16_t reg_data_high = (response.get_rx_data()[0] << 8) + response.get_rx_data()[1];
			stream_cache.read_stream_reg = combine_into_wide_register(reg_data_low, reg_data_high);
		}

		stream_cache.mode = response.get_rx_data()[4];
		uint16_t position_high = (response.get_rx_data()[5] << 8) | response.get_rx_data()[6];
		uint16_t position_low = (response.get_rx_data()[7] << 8) | response.get_rx_data()[8];
		stream_cache.position = combine_into_wide_register(position_low, position_high); 
		uint16_t force_high = (response.get_rx_data()[9] << 8) | response.get_rx_data()[10];
		uint16_t force_low = (response.get_rx_data()[11] << 8) | response.get_rx_data()[12];
		stream_cache.force = combine_into_wide_register(force_low, force_high);
		stream_cache.power = (response.get_rx_data()[13] << 8) | response.get_rx_data()[14];
		stream_cache.temperature = (response.get_rx_data()[15]);
		stream_cache.voltage = (response.get_rx_data()[16] << 8) | response.get_rx_data()[17];
		stream_cache.errors = (response.get_rx_data()[18] << 8) | response.get_rx_data()[19];
	}
		break;
	case motor_write: {
		stream_cache.mode = response.get_rx_data()[0];
		uint16_t position_high = (response.get_rx_data()[1] << 8) | response.get_rx_data()[2];
		uint16_t position_low = (response.get_rx_data()[3] << 8) | response.get_rx_data()[4];
		stream_cache.position = combine_into_wide_register(position_low, position_high);
		uint16_t force_high = (response.get_rx_data()[5] << 8) | response.get_rx_data()[6];
		uint16_t force_low = (response.get_rx_data()[7] << 8) | response.get_rx_data()[8];
		stream_cache.force = combine_into_wide_register(force_low, force_high);
		stream_cache.power = (response.get_rx_data()[9] << 8) | response.get_rx_data()[10];
		stream_cache.temperature = (response.get_rx_data()[11]);
		stream_cache.voltage = (response.get_rx_data()[12] << 8) | response.get_rx_data()[13];
		stream_cache.errors = (response.get_rx_data()[14] << 8) | response.get_rx_data()[15];
		break;
	}
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

OrcaResult<uint16_t> Actuator::get_mode_of_operation() {
	return read_register_blocking(MODE_OF_OPERATION);
}

OrcaResult<uint16_t> Actuator::get_power_W() {
	return read_register_blocking(POWER_REG_OFFSET);
}

OrcaResult<uint16_t> Actuator::get_temperature_C() {
	return read_register_blocking(TEMP_REG_OFFSET);
}

OrcaResult<uint16_t> Actuator::get_voltage_mV() {
	return read_register_blocking(VOLTAGE_REG_OFFSET);
}

OrcaResult<uint16_t> Actuator::get_errors() {
	return read_register_blocking(ERROR_REG_OFFSET);
}

OrcaResult<uint32_t> Actuator::get_serial_number() {
	auto [value, error] = read_wide_register_blocking(SERIAL_NUMBER_LOW);
	return { (uint32_t)value, error };
}

OrcaResult<uint16_t> Actuator::get_major_version() {
	return read_register_blocking(MAJOR_VERSION);
}

OrcaResult<uint16_t> Actuator::get_release_state() {
	return read_register_blocking(RELEASE_STATE);
}

OrcaResult<uint16_t> Actuator::get_revision_number() {
	return read_register_blocking(REVISION_NUMBER);
}

OrcaError Actuator::zero_position() {
	return write_register_blocking(ZERO_POS_REG_OFFSET, ZERO_POS_MASK);
}

OrcaError Actuator::clear_errors() {
	return write_register_blocking(CLEAR_ERROR_REG_OFFSET, CLEAR_ERROR_MASK);
}

OrcaResult<uint16_t> Actuator::get_latched_errors() {
	return read_register_blocking(ERROR_1);
}

OrcaError Actuator::set_max_force(s32 max_force) {
	return write_wide_register_blocking(USER_MAX_FORCE, max_force);
}

OrcaError Actuator::set_max_temp(uint16_t max_temp) {
	return write_register_blocking(USER_MAX_TEMP, max_temp);
}

OrcaError Actuator::set_max_power(uint16_t max_power) {
	return write_register_blocking(USER_MAX_POWER, max_power);
}

OrcaError Actuator::set_pctrl_tune_softstart(uint16_t t_in_ms) {
	return write_register_blocking(PC_SOFTSTART_PERIOD, t_in_ms);
}

OrcaError Actuator::set_safety_damping(uint16_t max_safety_damping) {
	return write_register_blocking(SAFETY_DGAIN, max_safety_damping);
}

//NEEDS TEST
void Actuator::tune_position_controller(uint16_t pgain, uint16_t igain, uint16_t dvgain, uint32_t sat, uint16_t degain) {

	uint16_t data[6] = {
		pgain,
		igain,
		dvgain,
		degain,
		uint16_t(sat),
		uint16_t(sat >> 16)
	};

	write_multiple_registers_blocking(PC_PGAIN, 6, data);
	write_register_blocking(CONTROL_REG_1::address, CONTROL_REG_1::position_controller_gain_set_flag);
}

//NEEDS TEST
OrcaError Actuator::set_kinematic_motion(int8_t ID, int32_t position, int32_t time, int16_t delay, int8_t type, int8_t auto_next, int8_t next_id) {
	if (next_id == -1) {
		next_id = ID + 1;
	}

	uint16_t data[6] = {
		uint16_t(position),
		uint16_t(position >> 16),
		uint16_t(time),
		uint16_t(time >> 16),
		uint16_t(delay),
		uint16_t((type << 1) | (next_id << 3) | auto_next)
	};
	return write_multiple_registers_blocking(KIN_MOTION_0 + (6 * ID), 6, data);
}

//NEEDS TEST
OrcaError Actuator::set_spring_effect(u8 spring_id, u16 gain, u32 center, u16 dead_zone, u16 saturation, u8 coupling) {
	uint16_t data[6] = {
		gain,
		uint16_t(center),
		uint16_t(center >> 16),
		coupling,
		dead_zone,
		saturation,

	};
	return write_multiple_registers_blocking(S0_GAIN_N_MM + spring_id * 6, 6, data);
}

//NEEDS TEST
OrcaError Actuator::set_osc_effect(u8 osc_id, u16 amplitude, u16 frequency_dhz, u16 duty, u16 type) {
	uint16_t data[4] = {
		amplitude,
		type,
		frequency_dhz,
		duty
	};
	return write_multiple_registers_blocking(O0_GAIN_N + osc_id * 4, 4, data);
}

OrcaError Actuator::set_damper(u16 damping) {
	return write_register_blocking(D0_GAIN_NS_MM, damping);
}

OrcaError Actuator::set_inertia(u16 inertia) {
	return write_register_blocking(I0_GAIN_NS2_MM, inertia);
}

OrcaError Actuator::set_constant_force(s32 force) {
	return write_wide_register_blocking(CONSTANT_FORCE_MN, force);
}

OrcaError Actuator::set_constant_force_filter(u16 force_filter) {
	return write_register_blocking(CONST_FORCE_FILTER, force_filter);
}

//NEEDS TEST: and command revisit
OrcaError Actuator::trigger_kinematic_motion(int8_t ID) {
	return write_register_blocking(KIN_SW_TRIGGER, ID);
}

OrcaError Actuator::begin_serial_logging(const std::string& log_name)
{
#ifdef WINDOWS
	std::shared_ptr<Log> app_log = std::make_shared<Log>();
	app_log->set_verbose_mode(false);
#endif
	return begin_serial_logging(log_name, app_log);
}

OrcaError Actuator::begin_serial_logging(const std::string& log_name, std::shared_ptr<LogInterface> log)
{
	OrcaError error = log->open(log_name);
	if (error) return error;
	modbus_client.begin_logging(log);
	return OrcaError(false, "");
}

void Actuator::set_stream_paused(bool paused)
{
	stream_paused = paused;
}

void Actuator::set_connection_config(ConnectionConfig config) {
	stream.set_connection_config(config);
}

void Actuator::enable_stream() {
	stream.enable();
}

void Actuator::disable_stream() {
	stream.disable();
	if (stream_is_established()) {
		stream.enqueue_change_connection_status_fn(modbus_server_address, false, 0, 0);
	}
	stream.disconnect();
}

bool Actuator::stream_is_established() {
	return stream.is_connected();
}
