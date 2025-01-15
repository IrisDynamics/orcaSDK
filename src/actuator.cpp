// TODO (Aiden Dec 20, 2024): Remove when making cross-platform
#include "platform/windows_serial_interface.h" 
#undef max
#undef min
#include "orca600_api/orca600.h"
#include "../actuator.h"
#include "chrono_clock.h"
#include "standard_modbus_functions.h"
#include "tools/log.h"
#include "command_and_confirm.h"
#include <limits>

int32_t combine_into_wide_register(uint16_t low_reg_value, uint16_t high_reg_value)
{
	return ((int32_t)high_reg_value << 16) + low_reg_value;
}

//Constructor
Actuator::Actuator(
	const char* name,
	uint8_t modbus_server_address
) :
	Actuator(
		std::make_shared<windows_SerialInterface>(),
		std::make_shared<ChronoClock>(),
		name,
		modbus_server_address
	)
{}

Actuator::Actuator(
	std::shared_ptr<SerialInterface> serial_interface,
	std::shared_ptr<Clock> clock,
	const char* name,
	uint8_t modbus_server_address
) :
	serial_interface(serial_interface),
	clock(clock),
	modbus_client(*serial_interface, *clock, 0),
	name(name),
	stream(this, modbus_client, modbus_server_address),
	modbus_server_address(modbus_server_address),
	_time_since_last_response_microseconds(std::numeric_limits<long long>::min() / 2) //Dividing by 2 because using minimum causes instant rollover
{}

OrcaError Actuator::open_serial_port(int port_number, int baud_rate, int interframe_delay) {
	serial_interface->set_new_serial_port(port_number);
	return modbus_client.init(baud_rate, interframe_delay);
}

void Actuator::close_serial_port() {
	serial_interface->close_serial_port();
}

int Actuator::channel_number() {
	return modbus_client.channel_number;
}

OrcaError Actuator::set_mode(MotorMode orca_mode) {
	bool command_success = command_and_confirm(*this, CTRL_REG_3, (uint16_t)orca_mode, MODE_OF_OPERATION, (uint16_t)orca_mode);
	if (!command_success) return OrcaError{ true, "Failed to set mode within 25ms!"};
	stream.update_motor_mode(orca_mode);
	return { false, "" };
}

OrcaResult<MotorMode> Actuator::get_mode() {
	auto return_struct = read_register_blocking(MODE_OF_OPERATION);
	return {(MotorMode)return_struct.value, return_struct.error};
}

void Actuator::set_streamed_force_mN(int32_t force) {
	stream.set_force_mN(force);
}

void Actuator::set_streamed_position_um(int32_t position) {
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

		handle_transaction_response(response);
	}
}

void Actuator::handle_transaction_response(Transaction response)
{
	message_data.clear();

	int ec = response.get_failure_codes();

	std::stringstream error_message;
	if (ec & (1 << Transaction::RESPONSE_TIMEOUT_ERROR)) error_message << "Response timed out, the motor took too long to respond. ";
	if (ec & (1 << Transaction::INTERCHAR_TIMEOUT_ERROR)) error_message << "Unexpected interchar delay timeout. ";
	if (ec & (1 << Transaction::UNEXPECTED_RESPONDER)) error_message << "Wrong modbus response address. ";
	if (ec & (1 << Transaction::CRC_ERROR)) error_message << "Wrong CRC. ";

	message_error = OrcaError{response.get_failure_codes(), error_message.str()};

	if (!ec)
	{
		_time_since_last_response_microseconds = clock->get_time_microseconds();
	}

	switch (response.get_rx_function_code()) {

	case ModbusFunctionCodes::read_holding_registers:
	case ModbusFunctionCodes::read_write_multiple_registers: {
		// add the received data to the local copy of the memory map
		//u16 register_start_address = (response.get_tx_data()[0] << 8) + response.get_tx_data()[1];
		uint16_t num_registers = (response.get_tx_data()[2] << 8) + response.get_tx_data()[3];
		for (int i = 0; i < num_registers; i++) {
			uint16_t register_data = (response.get_rx_data()[1 + i * 2] << 8) + response.get_rx_data()[2 + i * 2];
			message_data.push_back(register_data);
		}
		break;
	}
	case motor_command: {
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

OrcaResult<uint16_t> Actuator::get_power_W() {
	return read_register_blocking(POWER);
}

OrcaResult<uint16_t> Actuator::get_temperature_C() {
	return read_register_blocking(STATOR_TEMP);
}

OrcaResult<uint16_t> Actuator::get_voltage_mV() {
	return read_register_blocking(VDD_FINAL);
}

OrcaResult<uint16_t> Actuator::get_errors() {
	return read_register_blocking(ERROR_0);
}

OrcaResult<uint32_t> Actuator::get_serial_number() {
	OrcaResult<int32_t> result = read_wide_register_blocking(SERIAL_NUMBER_LOW);
	return { (uint32_t)result.value, result.error };
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
	return write_register_blocking(CTRL_REG_0, CONTROL_REG_0::position_zero_flag);
}

OrcaError Actuator::clear_errors() {
	return write_register_blocking(CTRL_REG_0, CONTROL_REG_0::clear_errors_flag);
}

OrcaResult<uint16_t> Actuator::get_latched_errors() {
	return read_register_blocking(ERROR_1);
}

OrcaError Actuator::set_max_force(int32_t max_force) {
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
OrcaError Actuator::set_spring_effect(uint8_t spring_id, uint16_t gain, int32_t center, uint16_t dead_zone, uint16_t saturation, uint8_t coupling) {
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
OrcaError Actuator::set_osc_effect(uint8_t osc_id, uint16_t amplitude, uint16_t frequency_dhz, uint16_t duty, OscillatorType type) {
	uint16_t data[4] = {
		amplitude,
		(uint16_t)type,
		frequency_dhz,
		duty
	};
	return write_multiple_registers_blocking(O0_GAIN_N + osc_id * 4, 4, data);
}

OrcaError Actuator::set_damper(uint16_t damping) {
	return write_register_blocking(D0_GAIN_NS_MM, damping);
}

OrcaError Actuator::set_inertia(uint16_t inertia) {
	return write_register_blocking(I0_GAIN_NS2_MM, inertia);
}

OrcaError Actuator::set_constant_force(int32_t force) {
	return write_wide_register_blocking(CONSTANT_FORCE_MN, force);
}

OrcaError Actuator::set_constant_force_filter(uint16_t force_filter) {
	return write_register_blocking(CONST_FORCE_FILTER, force_filter);
}

//NEEDS TEST: and command revisit
OrcaError Actuator::trigger_kinematic_motion(int8_t ID) {
	return write_register_blocking(KIN_SW_TRIGGER, ID);
}

OrcaError Actuator::begin_serial_logging(const std::string& log_name)
{
	std::shared_ptr<Log> app_log = std::make_shared<Log>();
	app_log->set_verbose_mode(false);
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

void Actuator::enable_stream() {
	stream.enable();
}

void Actuator::disable_stream() {
	stream.disable();
}

int64_t Actuator::time_since_last_response_microseconds()
{
	return clock->get_time_microseconds() - _time_since_last_response_microseconds;
}