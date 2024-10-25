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
	int uart_channel,
	const char* name,
	uint8_t modbus_server_address
) :
	Actuator(
#if defined(WINDOWS)
		std::make_shared<windows_SerialInterface>(uart_channel),
#endif
		std::make_shared<ChronoClock>(),
		uart_channel,
		name,
		modbus_server_address
	)
{}

Actuator::Actuator(
	std::shared_ptr<SerialInterface> serial_interface,
	std::shared_ptr<Clock> clock,
	int uart_channel,
	const char* name,
	uint8_t modbus_server_address
) :
	serial_interface(serial_interface),
	clock(clock),
	modbus_client(*serial_interface, *clock, uart_channel),
	name(name),
	stream(this, modbus_client, modbus_server_address),
	modbus_server_address(modbus_server_address)
{}

void Actuator::set_mode(MotorMode orca_mode) {
	write_register(CTRL_REG_3, (uint8_t)orca_mode);
	stream.update_stream_mode(orca_mode);
}

MotorMode Actuator::get_mode() {
	return (MotorMode)get_orca_reg_content(MODE_OF_OPERATION);
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

int32_t Actuator::get_force_mN() {
	return uint32_t(orca_reg_contents[FORCE_REG_H_OFFSET] << 16) | orca_reg_contents[FORCE_REG_OFFSET];
}

int32_t Actuator::get_position_um() {
	return (orca_reg_contents[POS_REG_H_OFFSET] << 16) | orca_reg_contents[POS_REG_OFFSET];
}

void Actuator::enable_haptic_effects(uint16_t effects) {
	write_register(HAPTIC_STATUS, effects);
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
	return orca_reg_contents[MODE_OF_OPERATION];
}

uint16_t Actuator::get_power_W() {
	return orca_reg_contents[POWER_REG_OFFSET];
}

uint16_t Actuator::get_temperature_C() {
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

void Actuator::set_kinematic_motion(int8_t ID, int32_t position, int32_t time, int16_t delay, int8_t type, int8_t auto_next, int8_t next_id) {
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

void Actuator::trigger_kinematic_motion(int8_t ID) {
	write_register(KIN_SW_TRIGGER, ID);
}

void Actuator::read_register(uint16_t reg_address, MessagePriority priority) {
	modbus_client.enqueue_transaction(DefaultModbusFunctions::read_holding_registers_fn(modbus_server_address, reg_address, 1, priority));
}

void Actuator::read_registers(uint16_t reg_address, uint8_t num_registers, MessagePriority priority) {
	if (num_registers < 1 || num_registers > MAX_NUM_READ_REG) return;

	modbus_client.enqueue_transaction(DefaultModbusFunctions::read_holding_registers_fn(modbus_server_address, reg_address, num_registers, priority));
}

void Actuator::write_register(uint16_t reg_address, uint16_t reg_data, MessagePriority priority) {
	modbus_client.enqueue_transaction(DefaultModbusFunctions::write_single_register_fn(modbus_server_address, reg_address, reg_data, priority));
}

void Actuator::write_registers(uint16_t reg_address, uint8_t num_registers, uint8_t* reg_data, MessagePriority priority) {
	if (num_registers < 1 || num_registers > MAX_NUM_WRITE_REG) return;

	modbus_client.enqueue_transaction(DefaultModbusFunctions::write_multiple_registers_fn(modbus_server_address, reg_address, num_registers, reg_data, priority));
}

void Actuator::write_registers(uint16_t reg_address, uint8_t num_registers, uint16_t* reg_data, MessagePriority priority) {
	if (num_registers < 1 || num_registers > MAX_NUM_WRITE_REG) return;

	uint8_t data[126];
	for (int i = 0; i < num_registers; i++) {
		data[i * 2] = uint8_t(reg_data[i] >> 8);
		data[i * 2 + 1] = uint8_t(reg_data[i]);
	}
	modbus_client.enqueue_transaction(DefaultModbusFunctions::write_multiple_registers_fn(modbus_server_address, reg_address, num_registers, data, priority));
}

void Actuator::read_write_registers(uint16_t read_starting_address, uint8_t read_num_registers, uint16_t write_starting_address, uint8_t write_num_registers, uint8_t* write_data, MessagePriority priority)
{
	if (read_num_registers < 1 || read_num_registers > MAX_NUM_READ_REG) return;
	if (write_num_registers < 1 || write_num_registers > MAX_NUM_WRITE_REG) return;

	modbus_client.enqueue_transaction(DefaultModbusFunctions::read_write_multiple_registers_fn(
		modbus_server_address,
		read_starting_address, read_num_registers,
		write_starting_address, write_num_registers,
		write_data,
		priority
	));
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

[[nodiscard("Ignored failure here will usually lead to an invalid application state")]]
bool Actuator::command_and_confirm(uint16_t command_register_address, uint16_t command_register_value, uint16_t confirm_register_address, uint16_t confirm_register_value)
{
	return command_and_confirm(
		command_register_address, 
		command_register_value, 
		confirm_register_address, 
		[this, confirm_register_address, confirm_register_value]()->bool { 
			return (get_orca_reg_content(confirm_register_address) == confirm_register_value); 
		});
}

[[nodiscard("Ignored failure here will usually lead to an invalid application state")]]
bool Actuator::command_and_confirm(uint16_t command_register_address, uint16_t command_register_value, uint16_t confirm_register_address, std::function<bool()> success_function)
{
	static constexpr int num_command_confirm_retries = 20;
	static constexpr int num_reads_per_command_retries = 3;

	bool command_was_successful = false;
	write_register(command_register_address, command_register_value);
	for (int i = 0; i < num_command_confirm_retries; i++)
	{
		if ((i % num_reads_per_command_retries) == 0) write_register(command_register_address, command_register_value);
		read_register(confirm_register_address);
		flush();
		if (success_function())
		{
			command_was_successful = true;
			break;
		}
	}
	return command_was_successful;
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
