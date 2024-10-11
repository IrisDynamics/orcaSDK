/**
   @file actuator.h
   @author Kali Erickson <kerickson@irisdynamics.com>, rebecca mcwilliam <rmcwilliam@irisdynamics.com>, kyle hagen <khagen@irisdynamics.com>, dan beddoes <dbeddoes@irisdynamics.com>
   @brief  Actuator object that abstracts the use of the modbus library/communications for communication with an Orca Series linear motor
   @version 2.2.0
    
    @copyright Copyright 2022 Iris Dynamics Ltd 
    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.

    For questions or feedback on this file, please email <support@irisdynamics.com>. 
 */

#ifndef ACTUATOR_H_
#define ACTUATOR_H_

#include "src/modbus_client_application.h"

#include "src/actuator_config.h"
#include "tools/Log.h"
#include <memory>


/**
   @class Actuator
   @brief Object that abstracts the communications between the client and a Orca motor server.
 */
class Actuator {

	std::shared_ptr<SerialInterface> serial_interface;
	std::shared_ptr<LogInterface> log;

public:
	ModbusClient modbus_client;

#if defined(WINDOWS)
	void set_new_comport(int _comport) {
		std::shared_ptr<windows_SerialInterface> win_modbus_client = std::dynamic_pointer_cast<windows_SerialInterface>(serial_interface);
		win_modbus_client->set_new_comport(_comport);
	}

	void disable_comport() {
		std::shared_ptr<windows_SerialInterface> win_modbus_client = std::dynamic_pointer_cast<windows_SerialInterface>(serial_interface);
		win_modbus_client->disable_comport_comms();
	}
#endif

public:
	//Constructor
	Actuator(
		int uart_channel,
		const char* name
	) :
		Actuator(
#if defined(WINDOWS)
			std::make_shared<windows_SerialInterface>(uart_channel),
			std::make_shared<Log>(),
#endif
			uart_channel,
			name
		)
	{}

	Actuator(
		std::shared_ptr<SerialInterface> serial_interface,
		std::shared_ptr<LogInterface> log,
		int uart_channel,
		const char* name
	) :
		serial_interface(serial_interface),
		log(log),
		modbus_client(*serial_interface, uart_channel),
		pause_time_cycles(DEFAULT_CONNECTION_PAUSE_uS)
	{}

	/**
	*@brief Sets the type of command that will be sent on high speed stream (ie when enable() has been used, this sets the type of message sent from enqueue motor frame)
	*/
	typedef enum {
		MotorCommand,
		MotorRead,
		MotorWrite
	}StreamMode;

	/**
	 * @brief this tracks the type of motor command stream that is currently being used
	 */
	typedef enum {

		SleepMode		= 1,
		ForceMode		= 2,
		PositionMode	= 3,
		HapticMode		= 4,
		KinematicMode	= 5

	} MotorMode;

	enum {
		ConstF	= 1 << 0,
		Spring0 = 1 << 1,
		Spring1 = 1 << 2,
		Spring2 = 1 << 3,
		Damper	= 1 << 4, 
		Inertia = 1 << 5,
		Osc0	= 1 << 6,
		Osc1	= 1	<< 7
	} HapticEffect;




	/**
	* @brief Write to the orca control register to change the mode of operation of the motor
	* note some modes require a constant stream to stay in that mode (eg. force, position)
	*/
	void set_mode(MotorMode orca_mode) {
		write_register(CTRL_REG_3, (uint8_t)orca_mode);
		comms_mode = orca_mode;
	}
	/**
	* @brief the communication mode determines which commands are sent by enqueue_motor_frame
	* *
	* @return CommunicationMode
	*/
	MotorMode get_mode() {
		return comms_mode;
	}

	/**
	* @brief Set the type of high speed stream to be sent on run out once handshake is complete
	*/

	void set_stream_mode(StreamMode mode) {
		stream_mode = mode;
	}

	/**	
	* @brief Get the current stream type to be sent on run out once handshake is complete
	*/
	StreamMode get_stream_mode() {
		return stream_mode;
	}

	/**
	* @brief This function can be continuously called and will update the values being sent when in motor write stream mode
	*/

	void update_write_stream(uint8_t width, uint16_t register_address, uint32_t register_value) {
		motor_write_data = register_value;
		motor_write_addr = register_address;
		motor_write_width = width;

	}

	/**
	* @brief This function can be continuously called and will update the values being sent when in motor read stream mode
	*/
	void update_read_stream(uint8_t width, uint16_t register_address) {
		motor_read_addr = register_address;
		motor_read_width = width;
	}


	/**
	* @brief Set/adjust the force that the motor is exerting when in motor_command stream mode
	* 
	* @param force force, in milli-Newtons
	*/
	void set_force_mN(int32_t force) {
		force_command = force;
		stream_timeout_start = modbus_client.get_system_cycles();
	}

	/**
	* @brief Set/adjust the position that the motor is aiming for when in motor command stream mode
	* 
	* @param position position, in micrometers
	*/
	void set_position_um(int32_t position) {
		position_command = position;
		stream_timeout_start = modbus_client.get_system_cycles();
	}

	/**
	* @brief Returns the total amount of force being sensed by the motor
	*
	* @return uint32_t - force in milli-Newtons
	*/
	int32_t get_force_mN() {
		return uint32_t(orca_reg_contents[FORCE_REG_H_OFFSET] << 16) | orca_reg_contents[FORCE_REG_OFFSET];
	}

	/**
	* @brief Returns the position of the shaft in the motor (distance from the zero position) in micrometers.
	*
	* @return uint32_t - position in micrometers
	*/
	int32_t get_position_um() {
		return (orca_reg_contents[POS_REG_H_OFFSET] << 16) | orca_reg_contents[POS_REG_OFFSET];
	}


	/**
	* @brief Enable or disabled desired haptic effects.
	*/
	void enable_haptic_effects(uint16_t effects) {
		write_register(HAPTIC_STATUS, effects);
	}

	/**
	* @brief Set the maximum time required between calls to set_force or set_position, in force or position mode respectively, before timing out and returning to sleep mode. 
	* 
	* @param timout_us time in microseconds
	*/
	void set_stream_timeout(uint64_t timeout_us){
		stream_timeout_cycles = timeout_us;
	}

	/**
	*@brief Get to a good handshake init state and set up the device driver with the default baud rate
	*/
	void init(){
		disconnect();	// dc is expected to return us to a good init state
		modbus_client.init(UART_BAUD_RATE);
	}

	/**
	 * @brief returns the number of successful messages between the actuator and the controller
	 * 
	 * @return uint16_t, number of successful messages
	 */
	uint16_t get_num_successful_msgs() {
		return success_msg_counter;
	}

	/**
	 * @brief returns the number of failed messages between the actuator and the controller
	 * 
	 * @return uint16_t, number of failed messages
	 */
	uint16_t	get_num_failed_msgs() {
		return failed_msg_counter;
	}


	void run()
	{
		run_out();
		run_in();
	}

	/**
	 * @brief process all commands in modbus queue
	 */
	void flush()
	{
		bool current_paused_state = stream_paused;
		set_stream_paused(true);

		while (modbus_client.get_queue_size() > 0)
		{
			run();
		}

		set_stream_paused(current_paused_state);
	}

	/**
	 * @brief handle the motor frame transmissions cadence
	 * @
	 * This dispatches transmissions for motor frames when connected and dispatches handshake messages when not.
	 * This function must be externally paced... i.e. called at the frequency that transmission should be sent
	 */
	void run_out() {
		handle_stream();
		// This function results in the UART sending any data that has been queued
		modbus_client.run_out();
	}


	/**
	 * @brief Incoming message parsing and connection handling
	 *
	 * Polls uart polled timers
	 * Claims responses from the message queue.
	 * Maintains the connection state based on consecutive failed messages
	 * Parses successful messages
	 */
	void run_in() {


		modbus_client.run_in();

		if ( modbus_client.is_response_ready() ) {
			Transaction response = modbus_client.dequeue_transaction();

			if (enabled && !stream_paused && !is_connected()) modbus_handshake(response);

			if ( !response.is_reception_valid() ) {
				cur_consec_failed_msgs++;
				failed_msg_counter++;
				if(connection_state == ConnectionStatus::connected && cur_consec_failed_msgs >= connection_config.max_consec_failed_msgs){
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

	void handle_transaction_response(Transaction response)
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

	/**
	 * @brief provide access to the modbus client isr function for linking to an interrupt controller
	 */
	void isr() {
#if defined(WINDOWS)
		std::shared_ptr<windows_SerialInterface> win_modbus_client = std::dynamic_pointer_cast<windows_SerialInterface>(serial_interface);
		win_modbus_client->uart_isr();
#endif
	}


	/**
	* @brief return the name of the actuator object 
	* 
	* @return char* - Name of the actuator object
	*/
	const char* get_name(){
		return my_name;
	}

	/**
	* @brief Returns the UART channel number in use
	* 
	* @return int, channel number
	*/
	int channel_number() {
		return modbus_client.channel_number;
	}

	/**
	*@brief get the motor's mode of operations as currently updated by the local memory map
	*/
	uint16_t get_mode_of_operation() {
		return orca_reg_contents[MODE_OF_OPERATION];
	}

	/**
	* @brief Returns the amount of power being drawn by the motor, in Watts
	* 
	* @return uint16_t - power in Watts
	*/
	uint16_t get_power_W(){
		return orca_reg_contents[POWER_REG_OFFSET];
	}

	/**
	* @brief Returns the temperature of the motor in Celcius
	* 
	* @return uint8_t - temperature in Celcius
	*/
	uint8_t get_temperature_C(){
		return orca_reg_contents[TEMP_REG_OFFSET];
	}

	/**
	* @brief Returns the amount of voltage the motor is recieving, in milli-Volts. 
	* 
	* @return uint16_t - voltage in milli-Voltage 
	*/
	uint16_t get_voltage_mV(){
		return orca_reg_contents[VOLTAGE_REG_OFFSET];
	}

	/**
	* @brief Returns the sum of all error messages being sent by the motor
	* 
	* @return uint16_t - sum or all active error codes
	*/
	uint16_t get_errors(){
		return orca_reg_contents[ERROR_REG_OFFSET];
	}

	/**
	* @brief Returns the actuator serial number
	* 
	* @return uint32_t - actuator serial number
	*/
	uint32_t get_serial_number() {

		uint16_t sn_high = orca_reg_contents[SERIAL_NUMBER_HIGH];
		uint16_t sn_low  = orca_reg_contents[SERIAL_NUMBER_LOW];

		uint32_t serial_number = (sn_high * (1<<16)) + sn_low;
		return serial_number;
	}

	/**
	* @brief Return the firmware major version
	* 
	* @return uint16_t - firmware major version
	*/
	uint16_t get_major_version() {
		return orca_reg_contents[MAJOR_VERSION];
	}

	/**
	* @brief Return the firmware release state (minor version)
	* 
	* @return uint16_t - firmware release state
	*/
	uint16_t get_release_state() {
		return orca_reg_contents[RELEASE_STATE];
	}

	/**
	* @brief Return the firmware revision number
	* 
	* @return uint16_t - firmware revision number
	*/
	uint16_t get_revision_number() {
		return orca_reg_contents[REVISION_NUMBER];
	}
	
	/**
	* @brief Returns true if the motor's firmware version is 'at least as recent' as the version designated 
	*		 by the parameters. 'At least as recent' can be thought of as a greater than or equal to comparison 
	*		 with version being the most significant digit, revision number being second most significant, and 
	*		 release state being the least significant.
	*
	* @param version - Desired major version number
	* @param release_state - Desired release state (0 - alpha, 1 - beta, 2 - release)
	* @param revision_number - Desired revision number
	* @return bool - True if motor's firmware version is at least as recent as the version designated by the parameters
	*/
	bool version_is_at_least(uint8_t version, uint8_t release_state, uint8_t revision_number) {
		return
			get_major_version() > version
			|| (get_major_version() == version && get_revision_number() > revision_number)
			|| (get_major_version() == version && get_revision_number() == revision_number && get_release_state() >= release_state);
	}

	/**
	 * @brief Set the zero position of the motor to be the current position 
	 */
	void zero_position(){
		write_register(ZERO_POS_REG_OFFSET, ZERO_POS_MASK);
	}

	/**
	 * @brief clear all errors stored on the motor
	 * note: errors that are still found will appear again
	 */
	void clear_errors(){
		write_register(CLEAR_ERROR_REG_OFFSET, CLEAR_ERROR_MASK);
	}

	/**
	 * @brief Copies the register for latched errors from the orca memory map into the local memory map 
	 * Latched errors are errors that were found by the motor, but are no longer active (not happening anymore)
	 */
	void get_latched_errors(){
		read_register(ERROR_1);
	}

	/**
	 * @brief Set the maximum force that the motor allows
	 * 
	 * @param max_force force in milli-Newtons
	 */
	void set_max_force(s32 max_force){
		uint8_t data[4] =	{
			uint8_t(max_force >> 8),
			uint8_t(max_force),
			uint8_t(max_force >> 24),
			uint8_t(max_force >> 16)
		};
		write_registers(USER_MAX_FORCE, 2, data);
	}

	/**
	 * @brief Set the maximum temperature that the motor allows
	 * 
	 * @param max_temp temperature in Celcius
	 */
	void set_max_temp(uint16_t max_temp){
		write_register(USER_MAX_TEMP  	,max_temp);
	}

	/**
	 * @brief Set the maximum power that the motor allows
	 * 
	 * @param max_power power in Watts
	 */
	void set_max_power(uint16_t max_power){
		write_register(USER_MAX_POWER 	,max_power);
	}

	/**
	 * @brief Sets the fade period when changing position controller tune in ms
	 * 
	 * @param t_in_ms time period in milliseconds
	*/
	void set_pctrl_tune_softstart(uint16_t t_in_ms){
		write_register(PC_SOFTSTART_PERIOD,t_in_ms);
	}

	/**
	 * @brief Sets the motion damping gain value used when communications are interrupted.
	 * 
	 * @param max_safety_damping damping value
	 */
	void set_safety_damping(uint16_t max_safety_damping){
		write_register(SAFETY_DGAIN 	,max_safety_damping);
	}

	/**
	 * @brief Sets the PID tuning values on the motor in non-scheduling mode. Disabled the gain scheduling in motors that support it
	 * 
	 * @param pgain proportional gain
	 * @param igain integral gain
	 * @param dgain derivative gain
	 * @param sat maximum force (safety value)
	 */
	void tune_position_controller(uint16_t pgain, uint16_t igain, uint16_t dvgain, uint32_t sat, uint16_t degain=0) {

		uint8_t data[12] = {
				uint8_t(pgain>>8),
				uint8_t(pgain),
				uint8_t(igain>>8),
				uint8_t(igain),
				uint8_t(dvgain>>8),
				uint8_t(dvgain),
				uint8_t(degain>>8),
				uint8_t(degain),
				uint8_t(sat>>8),
				uint8_t(sat),
				uint8_t(sat>>24),
				uint8_t(sat>>16)
		};

		write_registers(PC_PGAIN, 6,  data);
		write_register(CONTROL_REG_1::address, CONTROL_REG_1::position_controller_gain_set_flag);
	}

	/**
	* @brief Set the parameters to define a kinematic motion 
	* @param ID	Motion identifier
	* @param position Target position to reach
	* @param time	Time to get to the target
	* @param chain_delay	delay between this motion and the next
	* @param type	0 = minimize power, 1 = maximize smoothness
	* @param chain	Enable linking this motion to the next
	*/
	void set_kinematic_motion(int ID,int32_t position, int32_t time, int16_t delay, int8_t type, int8_t auto_next, int8_t next_id = -1) {
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
							uint8_t (0),
							uint8_t((type<<1) | (next_id << 3) | auto_next)
							};
		write_registers(KIN_MOTION_0 + (6*ID), 6, data);
	}

	/** @brief update the spring effect in a single function
	*/
	void set_spring_effect(u8 spring_id, u16 gain, u32 center, u16 dead_zone = 0, u16 saturation = 0, u8 coupling = 0) {
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

	/**
	*/
	void set_osc_effect(u8 osc_id, u16 amplitude, u16 frequency_dhz, u16 duty, u16 type) {
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


	/**
	* @brief Use the software trigger to start a kinematic motion, this will also run any chained motions 
	* @ID Identification of the motion to be triggered
	*/
	void trigger_kinematic_motion(int ID) {
		write_register(KIN_SW_TRIGGER, ID);
	}

	/**
	 * @brief Request for a specific register in the local copy to be updated from the motor's memory map
	 * 
	 * @param reg_address register address
	 */
	void read_register(uint16_t reg_address, MessagePriority priority = MessagePriority::important) {
		modbus_client.enqueue_transaction(DefaultModbusFunctions::read_holding_registers_fn(connection_config.server_address, reg_address, 1, priority));
	}
	
	/**
	 * @brief Request for multiple sequential registers in the local copy to be updated from the motor's memory map
	 *
	 * @param reg_address register address from the orca's memory map
	 * @param num_registers number of sequential registers to read
	 */
	void read_registers(uint16_t reg_address, uint16_t num_registers, MessagePriority priority = MessagePriority::important) {
		if(num_registers < 1 || num_registers > MAX_NUM_READ_REG) return;

		modbus_client.enqueue_transaction(DefaultModbusFunctions::read_holding_registers_fn(connection_config.server_address, reg_address, num_registers, priority));
	}

	/**
	 * @brief Request for a specific register in the motor's memory map to be updated with a given value.
	 * 
	 * @param reg_address register address
	 * @param reg_data data to be added to the register
	 */
	void write_register(uint16_t reg_address, uint16_t reg_data, MessagePriority priority = MessagePriority::important){
		modbus_client.enqueue_transaction(DefaultModbusFunctions::write_single_register_fn(connection_config.server_address, reg_address, reg_data, priority));
	}

	/**
	 * @brief Request for multiple registers in the motor's memory map to be updated with a given value.
	 *
	 * @param reg_address register address
	 * * @param num_registers number of sequential registers to write
	 * @param reg_data pointer to an array of data to be added to the registers
	 */
	void write_registers(uint16_t reg_address, uint16_t num_registers, uint8_t* reg_data, MessagePriority priority = MessagePriority::important) {
		if(num_registers < 1 || num_registers > MAX_NUM_WRITE_REG) return;

		modbus_client.enqueue_transaction(DefaultModbusFunctions::write_multiple_registers_fn(connection_config.server_address, reg_address, num_registers, reg_data, priority));
	}

	void write_registers(uint16_t reg_address, uint16_t num_registers, uint16_t* reg_data, MessagePriority priority = MessagePriority::important) {
		if(num_registers < 1 || num_registers > MAX_NUM_WRITE_REG) return;

		uint8_t data[126];
		for (int i = 0; i < num_registers; i++) {
			data[i*2] = reg_data[i] >> 8;
			data[i * 2 + 1] = reg_data[i];
		}
		modbus_client.enqueue_transaction(DefaultModbusFunctions::write_multiple_registers_fn(connection_config.server_address, reg_address, num_registers, data, priority));
	}

	/**
	 *	@brief Requests a read of multiple registers and also request a write of multiple registers
	 * 
	 *	@param read_starting_address The starting address of registers to read from
	 *  @param read_num_registers How many registers that should be read
	 *  @param write_starting_address The startin address of registers to write to
	 *  @param write_num_registers How many registers that should be written to
	 *  @param write_data Pointer to an array containing the byte data that should be written
	 */
	void read_write_registers(
		uint16_t read_starting_address, uint16_t read_num_registers,
		uint16_t write_starting_address, uint16_t write_num_registers,
		uint8_t* write_data,
		MessagePriority priority = MessagePriority::important)
	{
		if (read_num_registers < 1 || read_num_registers > MAX_NUM_READ_REG) return;
		if(write_num_registers < 1 || write_num_registers > MAX_NUM_WRITE_REG) return;

		modbus_client.enqueue_transaction(DefaultModbusFunctions::read_write_multiple_registers_fn(
			connection_config.server_address,
			read_starting_address, read_num_registers,
			write_starting_address, write_num_registers,
			write_data,
			priority
		));
	}

	/**
	* @brief Return the contents of the given register from the controller's copy of the motor's memory map. 
	* 
	* @param offset the register that will be read
	* @return uint16_t - register contents
	*/
	uint16_t get_orca_reg_content(uint16_t offset){
		return orca_reg_contents[offset];
	}

	void begin_serial_logging()
	{
#ifdef WINDOWS
		std::shared_ptr<Log> app_log = std::dynamic_pointer_cast<Log>(log);
		app_log->set_verbose_mode(false);
		app_log->open(my_name);
#endif
		modbus_client.begin_logging(log);
	}

	void set_stream_paused(bool paused)
	{
		stream_paused = paused;
	}

private:
	uint16_t orca_reg_contents[ORCA_REG_SIZE];

	StreamMode stream_mode = MotorCommand;
	MotorMode comms_mode = SleepMode;

	uint32_t stream_timeout_start;
	uint32_t stream_timeout_cycles = 100000;

	// Used to hold the last commanded force and position commands from the user of this object
	int32_t force_command;
	int32_t position_command;
	//Used to hold the last data to stream in motor write and read streams
	uint32_t motor_write_data = 0;
	uint16_t motor_write_addr = 0;
	uint16_t motor_write_width = 1;
	uint16_t motor_read_addr = 0;
	uint16_t motor_read_width = 1;

	bool stream_paused = false;

	// These counters are used to find the success and failure rate of the comms
	int32_t success_msg_counter = 0, failed_msg_counter = 0;

	/**
	 * @brief Requests the actuator synchronize its memory map with the controller
	 */
	void synchronize_memory_map() {
		read_registers(PARAM_REG_START     	, PARAM_REG_SIZE     			) ;
		read_registers(ERROR_0				, ADC_DATA_COLLISION-ERROR_0	) ;
		//read_registers(STATOR_CAL_REG_START	, STATOR_CAL_REG_SIZE			) ;
		//read_registers(SHAFT_CAL_REG_START 	, SHAFT_CAL_REG_SIZE 			) ;
		//read_registers(FORCE_CAL_REG_START 	, FORCE_CAL_REG_SIZE 			) ;
		read_registers(TUNING_REG_START    	, TUNING_REG_SIZE    			) ;
	}

	/**
	 * @brief Resets the memory map array to zeros
	 */
	void desynchronize_memory_map() {
		for(int i=0; i < ORCA_REG_SIZE; i++){
			orca_reg_contents[i] = 0;
		}
	}
#define KIN_CMD 32 // Number that indicates a kinematic type motor frame. Not an actual register like POS_CMD and FORCE_CMD
#define HAP_CMD 34
	void motor_stream_command() {
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
			motor_command_fn(connection_config.server_address, KIN_CMD, 0);
			break;
		case HapticMode:
			motor_command_fn(connection_config.server_address, HAP_CMD, 0);
			break;
		default:
			motor_command_fn(connection_config.server_address, 0, 0); //any register address other than force or position register_adresses will induce sleep mode and provided register_value will be ignored
			break;
		}
	}

	void motor_stream_read() {
		motor_read_fn(connection_config.server_address, motor_read_width, motor_read_addr);
	}

	void motor_stream_write() {
		motor_write_fn(connection_config.server_address, motor_write_width, motor_write_addr, motor_write_data);
	}

	/**
	 * @brief enqueue a motor message if the queue is empty
	 */
	void enqueue_motor_frame() {
		if(modbus_client.get_queue_size() >= 2) return;
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

	/**
	 * @brief Determine the length of the request for an application specific function code
	 * 
	 * @param fn_code application specific function code
	 * @return int - length of request
	 */
	int get_app_reception_length(uint8_t fn_code){
		switch(fn_code){
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

	/**
      @brief Enum of all actuator specific function codes, in decimal.
	 */
	enum orca_function_codes_e {
		motor_command = 100,
		motor_read = 104,
		motor_write = 105
	};

	/**
      @brief Format a motor command request, function code 0x64, and add the request to the buffer queue

      @param device_address Server device address
      @param command_code command code to specify command mode (sleep, force, position etc.)
      @param register_value The value to write to the register
	 */
	int motor_command_fn(uint8_t device_address, uint8_t command_code, int32_t register_value) {
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

	int motor_read_fn(uint8_t device_address, uint8_t width, uint16_t register_address) {
		uint8_t data_bytes[3] = {
				uint8_t(register_address>>8), 
				uint8_t(register_address),
				uint8_t(width)
		};

		Transaction my_temp_transaction;
		my_temp_transaction.load_transmission_data(device_address, motor_read, data_bytes, 3, get_app_reception_length(motor_read));
		int check = modbus_client.enqueue_transaction(my_temp_transaction);
		return check;
	}

	int motor_write_fn(uint8_t device_address, uint8_t width, uint16_t register_address, uint32_t register_value) {
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

#pragma region IRIS_CLIENT_APPLICATION
public:
	/**
	 * @brief Configurable parameters for the handshake sequence and connection maintenance. Should be set when disabled
	*/
	struct ConnectionConfig {
		uint8_t server_address = 1;
		int req_num_discovery_pings = 3; //3      //number of sucessful comms check messages required to move to next step in handshake sequence
		int max_consec_failed_msgs = 10;      //number of failed/missed messages to trigger disconnect
		uint32_t target_baud_rate_bps = 625000;
		uint16_t target_delay_us = 80;
		uint32_t response_timeout_us = 8000;  /// this timeout will be used to override the default response timeout after a handshake succeeds and a new baud rate is negotiated.
	};

	/**
	 * @brief Error check and apply the handshake/connection configuration parameters passed in the ConnectionConfig struct
	 *
	 * @param config ConnectionConfig object
	 * @return 0 if one of the parameters was invalid and default values were used, 1 otherwise
	*/
	int set_connection_config(Actuator::ConnectionConfig config) {

		if (config.server_address < 1 || config.server_address > 247) {
			return 0;   //todo : standardize error value
		}

		connection_config = config;

		return 1;   //todo : standardize error value
	}

	void handle_stream()
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

	/**
	 * @brief Determine if communication with a server is enabled or not
	 *
	 * @return boolean - the enabled status (true if enabled, false otherwise)
	*/
	bool is_enabled() {
		return enabled;
	}

	/**
	 * @brief Enable communication with a server device. Allows handshake sequence to begin, enables transceiver hardware
	*/
	void enable() {
		enabled = true;
	}

	/**
	 * @brief Disable communication with a server device. Moves into disconnecting state where transceiver hardware will be disabled
	*/
	void disable() {
		enabled = false;
		if (is_connected()) {
			enqueue_change_connection_status_fn(connection_config.server_address, false, 0, 0);
		}
		disconnect();
	}

	ConnectionConfig connection_config;



	/**
		* @brief Determine whether a server has successfully connected with this client
		* @return true if the server is in the connected state, false otherwise
	*/
	bool is_connected() {
		return connection_state == ConnectionStatus::connected;
	}

	/**
		* @brief Reset variables and move into the disconnected state
	*/
	void disconnect() {
		//reset states
		connection_state = ConnectionStatus::disconnected;
		cur_consec_failed_msgs = 0;
		modbus_client.adjust_baud_rate(UART_BAUD_RATE);
		modbus_client.adjust_response_timeout(DEFAULT_RESPONSE_uS);
		//is_paused = true;// pause to allow server to reset to disconnected state

		//start_pause_timer();
		desynchronize_memory_map();
	}
	enum class ConnectionStatus {
		disconnected,	// reset state
		discovery, 	// sending discovery pings, negotiating baud rate and delay
		synchronization,
		negotiation,
		connected,	// streaming commands to the server
	};

	volatile ConnectionStatus connection_state = ConnectionStatus::disconnected;

	void initiate_handshake()
	{
		if (modbus_client.get_queue_size() == 0 && has_pause_timer_expired()) {
			is_paused = false;
			new_data(); // clear new data flag
			num_discovery_pings_received = 0;
			enqueue_ping_msg();

			connection_state = ConnectionStatus::discovery;
		}
	}

	/**
		* @brief Perform the next step in the handshake routine with a server device.
		*
		* This function wants to progress from disconnected to connected through its various steps.
		* The state will remain in disconnected until the UARTs message queue is totally empty...ie all messages a received or timeout.
		* The state then becomes discovery where pings are sent until a number which is set in the config structure are successfully consecutively received
		* Following enough successful pings, we attempt to synchronize the server's memory map (if applicable) by queuing read register request(s).
		* If all requested read register messages are well received, a change connection status message is sent which requests the baud and interframe delay detailed by the config structure.
		* The state is now negotiation until the server responds. If the response is successful,
		*  the uart baud and interframe delays are adjusted based on what the server resports it realized
		*  and the state is now Connected.
		* If the negotiation fails, the state returns to discovery.
	*/
	void modbus_handshake(Transaction response) {

		switch (connection_state) {
		case ConnectionStatus::disconnected:
			initiate_handshake();
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

	void consume_new_message()
	{
		response = modbus_client.dequeue_transaction();
		new_data_flag = true;		// communicate to other layers that new data was received
	}
private:
	/**
	 * @brief Description of the possible connection states between the client and a server
	 *        Main state machine can be found in IrisClientApplication.
	 *        ModbusClient only responsible for moving to the 'disconnecting' state upon missed responses or errors
	*/
	const char* my_name;
	// Points to the last dequeued transaction from the modbus client
	Transaction response;

	// This is used to determine when a connection has terminated and the ConnectionStatus should change to disconnecting
	int cur_consec_failed_msgs = 0;          //!< current number of consecutive failed messages

	enum connection_function_codes_e {
		change_connection_status = 65
	};

	bool enabled = false;

	/**
	 * @brief Format a change_connection_status request, user-defined function code 65, and add the request to the buffer queue
	 * @param device_address
	 * @param connect true to connect the server, false to disconnect
	 * @param baud_rate_bps
	 * @param delay_us
	*/
	int enqueue_change_connection_status_fn(uint8_t device_address, bool connect, uint32_t baud_rate_bps, uint16_t delay_us) {
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

private:

	/**
	 * @brief returns true when a new message was parsed or has failed since the last time this was called and returned true
	 *
	 * This function must return true when a new transaction has been claimed
	 * since this function returned true last
	 */
	bool new_data() {
		bool return_value = new_data_flag;
		new_data_flag = false;
		return return_value;
	}

	volatile bool new_data_flag = false;

	int num_discovery_pings_received = 0;

	/////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////  Pause Timer  ///
	///////////////////////////////////////////////////////////////////////////

	bool is_paused = false;
	volatile uint32_t  pause_timer_start;
	const uint32_t pause_time_cycles;

	/**
	* @brief Start the pause timer. This can be done by saving the system time when the timer was started. Should not use interrupt timer
	*/
	virtual void start_pause_timer() {
		pause_timer_start = modbus_client.get_system_cycles();
		is_paused = 1;
	}
	/**
	* @brief Check the progress of the 200 millisecond interval (pause timer)
	* @return The remaining time in the interval, 0 if the interval is not set or has finished
	*/
	virtual bool has_pause_timer_expired() {
		if (!is_paused || (u32)(modbus_client.get_system_cycles() - pause_timer_start) >= pause_time_cycles) {//(pause_time_us*CYCLES_PER_MICROSECOND)){
			return 1;
		}
		return 0;
	}


	/**
	* @brief Format a Transaction to check the communication with a certain server
	* @return 1 if the request was added to the queue, 0 if the queue was full
	*
	*/
	int enqueue_ping_msg() {
		constexpr Transaction ping_transaction = DefaultModbusFunctions::return_query_data_fn(1/*connection_config.server_address*/); //pass in num_data as 0 so nothing from data array will be loaded into transmission
		return modbus_client.enqueue_transaction(ping_transaction);
	}


#pragma endregion


	////////////////////////////////////////////////////////////////////
	
public:
	//Deprecated. Just here for backwards compatibility. Placing down here because we do not want it used
	Actuator(
		int uart_channel,
		const char* name,
		int cycles_per_us
	) :
		Actuator(uart_channel, name)
	{}
};

#ifdef IRIS_ZYNQ_7000
extern Actuator actuator[6];
#endif

#endif

