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

#include "src/standard_modbus_functions.h"

#include "src/actuator_config.h"
#include "tools/log.h"
#include <memory>
#include <string>

#include <functional>
#include "src/orca_stream.h"

#include "src/orca_stream_config.h"
#include "src/orca_modes.h"

#include "src/error_types.h"

/**
   @class Actuator
   @brief Object that abstracts the communications between the client and a Orca motor server.
 */
class Actuator {

	std::shared_ptr<SerialInterface> serial_interface;
	std::shared_ptr<Clock> clock;

public:
	ModbusClient modbus_client;

public:
	/**
	 *	@brief	Constructs the object, if you don't know which constructor to use, use this one.
	 *	@param	serial_port_channel	The channel (E.g. 'COM_{X}') that the object should attempt to
	 *								obtain by default.
	 *	@param	name	The name of the object, can be obtained later through the 
						public member variable Actuator::name
	 */
	Actuator(
		int serial_port_channel,
		const char* name,
		uint8_t modbus_server_address = 1
	);

	/**
	 *	@brief	Constructs the object, passing in custom implementations for serial communication
	 *			and clocks. Useful for testing or for using this object on unsupported platforms.
	 */
	Actuator(
		std::shared_ptr<SerialInterface> serial_interface,
		std::shared_ptr<Clock> clock,
		int serial_port_channel,
		const char* name,
		uint8_t modbus_server_address = 1
	);

	const char* name;

	/**
	*@brief Get to a good handshake init state and set up the device driver with the default baud rate
	*/
	void init();

	/**
	* @brief Returns the total amount of force being sensed by the motor
	*
	* @return uint32_t - force in milli-Newtons
	*/
	OrcaResult<int32_t> get_force_mN();

	/**
	* @brief Returns the position of the shaft in the motor (distance from the zero position) in micrometers.
	*
	* @return uint32_t - position in micrometers
	*/
	OrcaResult<int32_t> get_position_um();

	/**
	*@brief get the motor's mode of operations as currently updated by the local memory map
	*/
	OrcaResult<uint16_t> get_mode_of_operation();

	/**
	* @brief Returns the sum of all error messages being sent by the motor
	* 
	* @return uint16_t - sum or all active error codes
	*/
	OrcaResult<uint16_t> get_errors();

	/**
	 * @brief clear all errors stored on the motor
	 * note: errors that are still found will appear again
	 */
	OrcaError clear_errors();

#if defined(WINDOWS)
	void set_new_serial_port(int port_number);

	void disable_serial_port();

	/**
	* @brief Returns the UART channel number in use
	*
	* @return int, channel number
	*/
	int channel_number();
#endif

#pragma region GENERIC_MODBUS_COMMUNICATION

	/**
	 * @brief Reads a double-wide register from the motor. 
	 *
	 * @param reg_address The lower register address of the double-wide register
	 */
	OrcaResult<int32_t> read_wide_register_blocking(uint16_t reg_address, MessagePriority priority = MessagePriority::important);
	
	/**
	 * @brief Reads a register from the motor.
	 *
	 * @param reg_address The register address
	 */
	OrcaResult<uint16_t> read_register_blocking(uint16_t reg_address, MessagePriority priority = MessagePriority::important);

	/**
	 * @brief Reads multiple registers from the motor.
	 *
	 * @param reg_start_address The starting register address
	 * @param num_registers How many registers to read
	 */
	OrcaResult<std::vector<uint16_t>> read_multiple_registers_blocking(uint16_t reg_start_address, uint8_t num_registers, MessagePriority priority = MessagePriority::important);

	/**
	 * @brief Writes a register from the motor.
	 *
	 * @param reg_address The register address
	 * @param write_data The value to be written
	 */
	OrcaError write_register_blocking(uint16_t reg_address, uint16_t write_data, MessagePriority priority = MessagePriority::important);

	/**
	 * @brief Writes a register to the motor.
	 *
	 * @param reg_address The lower address of the double-wide register 
	 * @param write_data The value to be written
	 */
	OrcaError write_wide_register_blocking(uint16_t reg_address, int32_t write_data, MessagePriority priority = MessagePriority::important);

	/**
	 * @brief Writes multiple register values to the motor.
	 *
	 * @param reg_start_address The starting register address to be written to
	 * @param num_registers How many registers to read
	 * @param write_data An array containing the value to be written
	 */
	OrcaError write_multiple_registers_blocking(uint16_t reg_start_address, uint8_t num_registers, uint16_t* write_data, MessagePriority priority = MessagePriority::important);

	/**
	 * @brief Simultaneously reads a set of values from the motor and writes a set of values to the motor.
	 *
	 * @param read_starting_address The starting register address to be read from
	 * @param read_num_registers The amount of registers to read
	 * @param write_starting_address The starting register address to be written to
	 * @param write_num_registers The amount of registers to write
	 * @param write_data An array containing the values to be written
	 */
	OrcaResult<std::vector<uint16_t>> read_write_multiple_registers_blocking(
		uint16_t read_starting_address, uint8_t read_num_registers,
		uint16_t write_starting_address, uint8_t write_num_registers,
		uint16_t* write_data,
		MessagePriority priority = MessagePriority::important);

	/**
	 *	@brief	Begins logging all serial communication between this application/object
	 *			and the motor that this application is talking to
	 *	@param	log_name	The name of the file to be written to. Assumes relative path.
	 */
	void begin_serial_logging(const std::string& log_name);
	void begin_serial_logging(const std::string& log_name, std::shared_ptr<LogInterface> log);

#pragma endregion

#pragma region STREAMING
	/**
	 *	@brief	The normal run loop for motor communication. Checks for incoming serial data and
	 *			sends queued serial data. If you are communicating with your motor asynchronously, you must
	 *			call this function in a regular loop. If you are using a high speed stream, and if
	 *			there are no queued messages, injects stream commands according to the motor mode
	 *			from the most recent call to set_mode().
	 */
	void run();

	/**
	 *	@brief	If called with parameter true, pauses automatic injection of stream
	 *			commands during calls to run()
	 */
	void set_stream_paused(bool paused);

	/**
	* @brief Set/adjust the force that the motor is exerting when in motor_command stream mode
	*
	* @param force force, in milli-Newtons
	*/
	void set_force_mN(int32_t force);

	/**
	* @brief Set/adjust the position that the motor is aiming for when in motor command stream mode
	*
	* @param position position, in micrometers
	*/
	void set_position_um(int32_t position);

	/**
	* @brief Update which haptic effects will be set through the motor command frame
	*
	* @param effects The bitmap describing which haptic effects should be enabled and disabled
	*/
	void update_haptic_stream_effects(uint16_t effects);

	/**
	* @brief	Write to the orca control register to change the mode of operation of the motor.
	*			Also changes what type of command stream will be sent during high speed streaming.
	*/
	OrcaError set_mode(MotorMode orca_mode);

	/**
	* @brief the communication mode determines which commands are sent by enqueue_motor_frame
	* *
	* @return CommunicationMode
	*/
	OrcaResult<MotorMode> get_mode();

	/**
	* @brief Set the type of high speed stream to be sent on run out once handshake is complete
	*/
	void set_stream_mode(OrcaStream::StreamMode mode);

	/**
	* @brief This function can be continuously called and will update the values being sent when in motor write stream mode
	*/
	void update_write_stream(uint8_t width, uint16_t register_address, uint32_t register_value);

	/**
	* @brief This function can be continuously called and will update the values being sent when in motor read stream mode
	*/
	void update_read_stream(uint8_t width, uint16_t register_address);

	/**
	 * @brief Error check and apply the handshake/connection configuration parameters passed in the ConnectionConfig struct
	 *
	 * @param config ConnectionConfig object
	 * @return 0 if one of the parameters was invalid and default values were used, 1 otherwise
	*/
	void set_connection_config(ConnectionConfig config);

	/**
	 * @brief Enable communication with a server device. Allows handshake sequence to begin, enables transceiver hardware
	*/
	void enable();

	/**
	 * @brief Disable communication with a server device. Moves into disconnecting state where transceiver hardware will be disabled
	*/
	void disable();

	/**
		* @brief Determine whether the Actuator object has successfully initiated a stream with the motor
		* @return true if the stream is in the connected state, false otherwise
	*/
	bool is_connected();

#pragma endregion

#pragma region UNCOMMON_MISC_DATA

	/**
	* @brief Returns the amount of power being drawn by the motor, in Watts
	* 
	* @return uint16_t - power in Watts
	*/
	OrcaResult<uint16_t> get_power_W();

	/**
	* @brief Returns the temperature of the motor in Celcius
	* 
	* @return uint16_t - temperature in Celcius
	*/
	OrcaResult<uint16_t> get_temperature_C();

	/**
	* @brief Returns the amount of voltage the motor is recieving, in milli-Volts. 
	* 
	* @return uint16_t - voltage in milli-Voltage 
	*/
	OrcaResult<uint16_t> get_voltage_mV();

	/**
	 * @brief Set the zero position of the motor to be the current position 
	 */
	OrcaError zero_position();

	/**
	 * @brief Copies the register for latched errors from the orca memory map into the local memory map 
	 * Latched errors are errors that were found by the motor, but are no longer active (not happening anymore)
	 */
	OrcaResult<uint16_t> get_latched_errors();

#pragma endregion

#pragma region MOTOR_ID_AND_VERSIONING

	/**
	* @brief Returns the actuator serial number
	*
	* @return uint32_t - actuator serial number
	*/
	OrcaResult<uint32_t> get_serial_number();

	/**
	* @brief Return the firmware major version
	*
	* @return uint16_t - firmware major version
	*/
	OrcaResult<uint16_t> get_major_version();

	/**
	* @brief Return the firmware release state (minor version)
	*
	* @return uint16_t - firmware release state
	*/
	OrcaResult<uint16_t> get_release_state();

	/**
	* @brief Return the firmware revision number
	*
	* @return uint16_t - firmware revision number
	*/
	OrcaResult<uint16_t> get_revision_number();

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
	OrcaResult<bool> version_is_at_least(uint8_t version, uint8_t release_state, uint8_t revision_number);

#pragma endregion

#pragma region TUNING_AND_SAFETY

	/**
	 * @brief Set the maximum force that the motor allows
	 * 
	 * @param max_force force in milli-Newtons
	 */
	OrcaError set_max_force(s32 max_force);

	/**
	 * @brief Set the maximum temperature that the motor allows
	 * 
	 * @param max_temp temperature in Celcius
	 */
	OrcaError set_max_temp(uint16_t max_temp);

	/**
	 * @brief Set the maximum power that the motor allows
	 * 
	 * @param max_power power in Watts
	 */
	OrcaError set_max_power(uint16_t max_power);

	/**
	 * @brief Sets the fade period when changing position controller tune in ms
	 * 
	 * @param t_in_ms time period in milliseconds
	*/
	OrcaError set_pctrl_tune_softstart(uint16_t t_in_ms);

	/**
	 * @brief Sets the motion damping gain value used when communications are interrupted.
	 * 
	 * @param max_safety_damping damping value
	 */
	OrcaError set_safety_damping(uint16_t max_safety_damping);

	/**
	 * @brief Sets the PID tuning values on the motor in non-scheduling mode. Disabled the gain scheduling in motors that support it
	 * 
	 * @param pgain proportional gain
	 * @param igain integral gain
	 * @param dgain derivative gain
	 * @param sat maximum force (safety value)
	 */
	void tune_position_controller(uint16_t pgain, uint16_t igain, uint16_t dvgain, uint32_t sat, uint16_t degain=0);

#pragma endregion

#pragma region KINEMATICS

	/**
	* @brief Set the parameters to define a kinematic motion 
	* @param ID	Motion identifier
	* @param position Target position to reach
	* @param time	Time to get to the target
	* @param chain_delay	delay between this motion and the next
	* @param type	0 = minimize power, 1 = maximize smoothness
	* @param chain	Enable linking this motion to the next
	*/
	OrcaError set_kinematic_motion(int8_t ID,int32_t position, int32_t time, int16_t delay, int8_t type, int8_t auto_next, int8_t next_id = -1);

	/**
	* @brief Use the software trigger to start a kinematic motion, this will also run any chained motions
	* @ID Identification of the motion to be triggered
	*/
	OrcaError trigger_kinematic_motion(int8_t ID);

#pragma endregion

#pragma region HAPTICS

	enum HapticEffect {
		ConstF = 1 << 0,
		Spring0 = 1 << 1,
		Spring1 = 1 << 2,
		Spring2 = 1 << 3,
		Damper = 1 << 4,
		Inertia = 1 << 5,
		Osc0 = 1 << 6,
		Osc1 = 1 << 7
	};

	/**
	* @brief	Sets each haptic effect to enabled or disabled according to the input bits.
	 *	@notes	Please refer to the Orcs Series Reference Manual, section Controllers->Haptic Controller
	 *			for details on this function.
	*/
	OrcaError enable_haptic_effects(uint16_t effects);

	/** 
	 *	@brief	Configures a spring effect with the given parameters.
	 *	@notes	Please refer to the Orcs Series Reference Manual, section Controllers->Haptic Controller
	 *			for details on this function.
	 */
	OrcaError set_spring_effect(u8 spring_id, u16 gain, u32 center, u16 dead_zone = 0, u16 saturation = 0, u8 coupling = 0);

	/**
	 *	@brief	Configures the parameters of an oscillation effect with the given parameters.
	 *	@notes	Please refer to the Orcs Series Reference Manual, section Controllers->Haptic Controller
	 *			for details on this function.
	 * 
	 */
	OrcaError set_osc_effect(u8 osc_id, u16 amplitude, u16 frequency_dhz, u16 duty, u16 type);

	/**
	*	@brief Sets the damping value in Haptic Mode
	*	@param damping	The damping gain (4*N*s/mm)
	*/
	void set_damper(u16 damping);

	/**
	*	@brief Sets the inertia value in Haptic Mode
	*	@param inertia	The inertia gain (64*N*s^2/mm)
	*/
	void set_inertia(u16 inertia);

	/**
	*	@brief Sets the constant force value in Haptic Mode
	*	@param force	(mN)
	*/
	void set_constant_force(s32 force);

	/**
	*	@brief Sets the constant force filter value in Haptic Mode
	*	@param force_filter	Amount of filtering on constant force inputs
	*/
	void set_constant_force_filter(u16 force_filter);

#pragma endregion

	struct StreamReturnData
	{
		int32_t read_stream_reg{ 0 };
		uint16_t mode{ 0 };
		int32_t position{ 0 };
		int32_t force{ 0 };
		uint16_t power{ 0 };
		int16_t temperature{ 0 };
		uint16_t voltage{ 0 };
		uint16_t errors{ 0 };
	};

	StreamReturnData stream_cache;

private:
	OrcaError message_error{false};
	std::vector<uint16_t> message_data{};
	
	OrcaStream stream;

	const uint8_t modbus_server_address;

	void handle_transaction_response(Transaction response);

	/**
	 * @brief handle the motor frame transmissions cadence
	 * @
	 * This dispatches transmissions for motor frames when connected and dispatches handshake messages when not.
	 * This function must be externally paced... i.e. called at the frequency that transmission should be sent
	 */
	void run_out();

	/**
	 * @brief Incoming message parsing and connection handling
	 *
	 * Polls uart polled timers
	 * Claims responses from the message queue.
	 * Maintains the connection state based on consecutive failed messages
	 * Parses successful messages
	 */
	void run_in();

	/**
	 * @brief	Flushes all queued messages, blocking the current thread of execution
	 *			until all queued messages have completed.
	 * @note	While this function blocks until all messages complete, it doesn't necessarily
	 *			block until all commands are fully processed. For example, a mode change
	 *			command may be acknowledged by the motor without being immediately processed.
	 *			This acknowledgement will be considered a completed message by this client.
	 *			To block until commands are successfully processed, additional post-conditions
	 *			should be checked. See command_and_confirm() for an alternative to flush() in
	 *			this case.
	 * @note	A completed message does not mean a successful message. A message may
	 *			be considered complete if it times out due to too much delay between
	 *			sending a message and receiving a response, for example. See modbus documentation
	 *			for a list of failure types.
	 */
	void flush();

	bool stream_paused = false;
	
public:
	[[deprecated("Requests initialization of now unused parameters")]]
	Actuator(
		int uart_channel,
		const char* name,
		int
	) :
		Actuator(uart_channel, name)
	{}
};

#endif