#include "../actuator.h"
#include "chrono_clock.h"

#if defined(WINDOWS)
void Actuator::set_new_comport(int _comport) {
	std::shared_ptr<windows_SerialInterface> win_modbus_client = std::dynamic_pointer_cast<windows_SerialInterface>(serial_interface);
	win_modbus_client->set_new_comport(_comport);
}

void Actuator::disable_comport() {
	std::shared_ptr<windows_SerialInterface> win_modbus_client = std::dynamic_pointer_cast<windows_SerialInterface>(serial_interface);
	win_modbus_client->disable_comport_comms();
}

int Actuator::channel_number() {
	return modbus_client.channel_number;
}
#endif

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

void Actuator::set_mode(MotorMode orca_mode) {
	write_register_blocking(CTRL_REG_3, (uint16_t)orca_mode);

	if (get_mode() != orca_mode)
	{
		std::cout << "ERROR: Failed to switch mode during set_mode()\n";
	}

	stream.update_stream_mode(orca_mode);
}

MotorMode Actuator::get_mode() {
	return (MotorMode)read_register_blocking(MODE_OF_OPERATION);
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

int32_t Actuator::read_wide_register_blocking(uint16_t reg_address, MessagePriority priority)
{
	modbus_client.enqueue_transaction(DefaultModbusFunctions::read_holding_registers_fn(modbus_server_address, reg_address, 2, priority));
	flush();
	return ((int32_t)orca_reg_contents[reg_address + 1] << 16) + orca_reg_contents[reg_address];
}

uint16_t Actuator::read_register_blocking(uint16_t reg_address, MessagePriority priority)
{
	modbus_client.enqueue_transaction(DefaultModbusFunctions::read_holding_registers_fn(modbus_server_address, reg_address, 1, priority));
	flush();
	return orca_reg_contents[reg_address];
}

std::vector<uint16_t> Actuator::read_multiple_registers_blocking(uint16_t reg_start_address, uint8_t num_registers, MessagePriority priority)
{
	if (num_registers == 0) return {};

	modbus_client.enqueue_transaction(DefaultModbusFunctions::read_holding_registers_fn(modbus_server_address, reg_start_address, num_registers, priority));
	flush();
	std::vector<uint16_t> output_vec;
	for (size_t i = 0; i < num_registers; i++)
	{
		output_vec.push_back(orca_reg_contents[reg_start_address + i]);
	}
	return output_vec;
}

void Actuator::write_register_blocking(uint16_t reg_address, uint16_t write_data, MessagePriority priority)
{
	modbus_client.enqueue_transaction(DefaultModbusFunctions::write_single_register_fn(modbus_server_address, reg_address, write_data, priority));
	flush();
}

void Actuator::write_wide_register_blocking(uint16_t reg_address, int32_t write_data, MessagePriority priority)
{
	uint16_t split_data[2]{
		uint16_t(write_data),
		uint16_t(write_data >> 16)
	};
	write_multiple_registers_blocking(reg_address, 2, split_data, priority);
}

void Actuator::write_multiple_registers_blocking(uint16_t reg_start_address, uint8_t num_registers, uint16_t* write_data, MessagePriority priority)
{
	if (num_registers == 0) return;

	uint8_t data[128];
	for (int i = 0; i < num_registers; i++) {
		data[i * 2] = uint8_t(write_data[i] >> 8);
		data[i * 2 + 1] = uint8_t(write_data[i]);
	}
	modbus_client.enqueue_transaction(DefaultModbusFunctions::write_multiple_registers_fn(modbus_server_address, reg_start_address, num_registers, data, priority));
	flush();
}

std::vector<uint16_t> Actuator::read_write_multiple_registers_blocking(
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

	std::vector<uint16_t> output_vec;
	for (size_t i = 0; i < read_num_registers; i++)
	{
		output_vec.push_back(orca_reg_contents[read_starting_address + i]);
	}

	return output_vec;
}

int32_t Actuator::get_force_mN() {
	return read_wide_register_blocking(FORCE);
}

int32_t Actuator::get_position_um() {
	return read_wide_register_blocking(SHAFT_POS_UM);
}

void Actuator::update_haptic_stream_effects(uint16_t effects) {
	stream.set_haptic_effects(effects);
}

void Actuator::enable_haptic_effects(uint16_t effects) {
	write_register_blocking(HAPTIC_STATUS, effects);
}

void Actuator::set_stream_timeout(int64_t timeout_us) {
	stream.set_stream_timeout(timeout_us);
}

void Actuator::init() {
	stream.disconnect();	// dc is expected to return us to a good init state
	desynchronize_memory_map();
	modbus_client.init(UART_BAUD_RATE);
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

		if (stream.is_enabled() && !stream_paused && !is_connected()) stream.modbus_handshake(response);

		stream.update_stream_state(response);

		if (response.is_reception_valid()) {
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

uint16_t Actuator::get_mode_of_operation() {
	return read_register_blocking(MODE_OF_OPERATION);
}

uint16_t Actuator::get_power_W() {
	return read_register_blocking(POWER_REG_OFFSET);
}

uint16_t Actuator::get_temperature_C() {
	return read_register_blocking(TEMP_REG_OFFSET);
}

uint16_t Actuator::get_voltage_mV() {
	return read_register_blocking(VOLTAGE_REG_OFFSET);
}

uint16_t Actuator::get_errors() {
	return read_register_blocking(ERROR_REG_OFFSET);
}

uint32_t Actuator::get_serial_number() {
	return (uint32_t)read_wide_register_blocking(SERIAL_NUMBER_LOW);
}

uint16_t Actuator::get_major_version() {
	return read_register_blocking(MAJOR_VERSION);
}

uint16_t Actuator::get_release_state() {
	return read_register_blocking(RELEASE_STATE);
}

uint16_t Actuator::get_revision_number() {
	return read_register_blocking(REVISION_NUMBER);
}

bool Actuator::version_is_at_least(uint8_t version, uint8_t release_state, uint8_t revision_number) {
	std::vector<uint16_t> version_registers = read_multiple_registers_blocking(MAJOR_VERSION, 3);
	
	uint16_t read_major_version = version_registers[0];
	uint16_t read_revision_number = version_registers[2];
	uint16_t read_release_state = version_registers[1];

	return
		read_major_version > version
		|| (read_major_version == version && read_revision_number > revision_number)
		|| (read_major_version == version && read_revision_number == revision_number && read_release_state >= release_state);
}

void Actuator::zero_position() {
	write_register_blocking(ZERO_POS_REG_OFFSET, ZERO_POS_MASK);
}

void Actuator::clear_errors() {
	write_register_blocking(CLEAR_ERROR_REG_OFFSET, CLEAR_ERROR_MASK);
}

uint16_t Actuator::get_latched_errors() {
	return read_register_blocking(ERROR_1);
}

void Actuator::set_max_force(s32 max_force) {
	write_wide_register_blocking(USER_MAX_FORCE, max_force);
}

void Actuator::set_max_temp(uint16_t max_temp) {
	write_register_blocking(USER_MAX_TEMP, max_temp);
}

void Actuator::set_max_power(uint16_t max_power) {
	write_register_blocking(USER_MAX_POWER, max_power);
}

void Actuator::set_pctrl_tune_softstart(uint16_t t_in_ms) {
	write_register_blocking(PC_SOFTSTART_PERIOD, t_in_ms);
}

void Actuator::set_safety_damping(uint16_t max_safety_damping) {
	write_register_blocking(SAFETY_DGAIN, max_safety_damping);
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
void Actuator::set_kinematic_motion(int8_t ID, int32_t position, int32_t time, int16_t delay, int8_t type, int8_t auto_next, int8_t next_id) {
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
	write_multiple_registers_blocking(KIN_MOTION_0 + (6 * ID), 6, data);
}

//NEEDS TEST
void Actuator::set_spring_effect(u8 spring_id, u16 gain, u32 center, u16 dead_zone, u16 saturation, u8 coupling) {
	uint16_t data[6] = {
		gain,
		uint16_t(center),
		uint16_t(center >> 16),
		coupling,
		dead_zone,
		saturation,

	};
	write_multiple_registers_blocking(S0_GAIN_N_MM + spring_id * 6, 6, data);
}

//NEEDS TEST
void Actuator::set_osc_effect(u8 osc_id, u16 amplitude, u16 frequency_dhz, u16 duty, u16 type) {
	uint16_t data[4] = {
		amplitude,
		type,
		frequency_dhz,
		duty
	};
	write_multiple_registers_blocking(O0_GAIN_N + osc_id * 4, 4, data);
}

void Actuator::set_damper(u16 damping) {
	write_register_blocking(D0_GAIN_NS_MM, damping);
}

void Actuator::set_inertia(u16 inertia) {
	write_register_blocking(I0_GAIN_NS2_MM, inertia);
}

void Actuator::set_constant_force(s32 force) {
	write_wide_register_blocking(CONSTANT_FORCE_MN, force);
}

void Actuator::set_constant_force_filter(u16 force_filter) {
	write_register_blocking(CONST_FORCE_FILTER, force_filter);
}

//NEEDS TEST: and command revisit
void Actuator::trigger_kinematic_motion(int8_t ID) {
	write_register_blocking(KIN_SW_TRIGGER, ID);
}

uint16_t Actuator::get_orca_reg_content(uint16_t offset) {
	return orca_reg_contents[offset];
}

void Actuator::begin_serial_logging(const std::string& log_name)
{
#ifdef WINDOWS
	std::shared_ptr<Log> app_log = std::make_shared<Log>();
	app_log->set_verbose_mode(false);
#endif
	begin_serial_logging(log_name, app_log);
}

void Actuator::begin_serial_logging(const std::string& log_name, std::shared_ptr<LogInterface> log)
{
	log->open(log_name);
	modbus_client.begin_logging(log);
}

void Actuator::set_stream_paused(bool paused)
{
	stream_paused = paused;
}

void Actuator::desynchronize_memory_map() {
	orca_reg_contents.fill(0);
}

void Actuator::set_connection_config(ConnectionConfig config) {
	stream.set_connection_config(config);
}

void Actuator::enable() {
	stream.enable();
}

void Actuator::disable() {
	stream.disable();
	if (is_connected()) {
		stream.enqueue_change_connection_status_fn(modbus_server_address, false, 0, 0);
	}
	stream.disconnect();
}

bool Actuator::is_connected() {
	return stream.is_connected();
}
