/**
   @file actuator.h
   @author Aiden Bull <abull@irisdynamics.com>, Kali Erickson <kerickson@irisdynamics.com>, rebecca mcwilliam <rmcwilliam@irisdynamics.com>, kyle hagen <khagen@irisdynamics.com>, dan beddoes <dbeddoes@irisdynamics.com>
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

#include <cstdint>
#include "src/message_priority.h"
extern "C" {
	#include "c/orca_registers.h"
}
#include "cpp/orca_registers.hpp"
#include <string>
#include <vector>

#include <functional>
#include "src/serial_interface.h"
#include "src/modbus_client.h"
#include "src/orca_stream.h"
#include "src/orca_modes.h"

#include "src/error_types.h"
#include "src/command_stream_structs.h"
#include "src/constants.h"

namespace orcaSDK
{

/**
   @class Actuator
   @brief Class that abstracts the communications between the client and an ORCA Series Linear Motor.

		This class should be the centerpoint for all communication with your motor from this 
		SDK. Other classes and types in this SDK should be understood in reference to this class.
 */
class Actuator {

	std::shared_ptr<SerialInterface> serial_interface;
	std::shared_ptr<Clock> clock;

public:
	ModbusClient modbus_client;

public:
	/**
	 *	@brief	Constructs the object, if you don't know which constructor to use, use this one.
	 *	@param	name	The name of the object, can be obtained later through the 
	 *					public member variable Actuator::name
	 *	@param	modbus_server_address	The modbus address of the server that should be communicated
	 *									with. See modbus specification for details.
	 *	@note	If you don't know which constructor to use, use this one.
	 *	@overload Actuator::Actuator(const char* name, uint8_t modbus_server_address = 1)
	 */
	Actuator(
		const char* name = "",
		uint8_t modbus_server_address = 1
	);

	/**
	 *	@brief	Constructs the object, passing in custom implementations for serial communication
	 *			and clocks. Useful for testing or for using this object on unsupported platforms.
	 *	@param	serial_interface	A custom implementation of the SerialInterface pure virtual class
	 *	@param	clock	A custom implementation of the Clock pure virtual class
	 *	@param	name	The name of the object, can be obtained later through the 
	 *					public member variable Actuator::name
	 *	@param	modbus_server_address	The modbus address of the server that should be communicated
	 *									with. See modbus specification for details.
	 *	@note	Use this constructor if you need an implementation of this object for a platform that we don't yet support.
	 *	@overload Actuator::Actuator(std::shared_ptr<SerialInterface> serial_interface, std::shared_ptr<Clock> clock, const char* name, uint8_t modbus_server_address = 1)
	 */
	Actuator(
		std::shared_ptr<SerialInterface> serial_interface,
		std::shared_ptr<Clock> clock,
		const char* name = "",
		uint8_t modbus_server_address = 1
	);

	std::string name;

	/**
	 * @brief	Attempts to open a desired serial port, assigning a baud rate and interframe
	 *			delay for this connection.
	 * @param	port_path	The file path corresponding to the rs422 cable that the desired device
	 *						 is connected to.
	 * @param	baud_rate	The desired baud rate for this connection. Should match the baud
	 *						rate configured in the ORCA's modbus settings.
	 * @param	interframe_delay	The time that this object will wait after concluding a message
	 *								before initiating the next. Should match the interframe delay
	 *								configured in the ORCA's modbus settings.
	 */
	OrcaError open_serial_port(
		std::string port_path,
		int baud_rate = Constants::kDefaultBaudRate,
		int interframe_delay = Constants::kDefaultInterframeDelay_uS
	);

	/**
	 *	Attempts to open a desired serial port, creating the expected file path
	 *	from the assigned port number of the Actuator's rs422 cable.
	 * @param	port_number	The assigned port number of the rs422 cable that the desired device
	 *						 is connected to.
	 * @overload	Actuator::open_serial_port(int port_number, int baud_rate, int interframe_delay)
	 */
	OrcaError open_serial_port(
		int port_number, 
		int baud_rate = Constants::kDefaultBaudRate,
		int interframe_delay = Constants::kDefaultInterframeDelay_uS
	);

	/**
	 *	@brief	Closes any open serial port and releases all handles to it.
	 */
	void close_serial_port();

	/**
	* @brief Returns the total amount of force being sensed by the motor
	*
	* @return int32_t  Force in milli-Newtons
	*/
	OrcaResult<int32_t> get_force_mN();

	/**
	* @brief Returns the position of the shaft in the motor (distance from the zero position) in micrometers.
	*
	* @return uint32_t  Position in micrometers
	*/
	OrcaResult<int32_t> get_position_um();

	/**
	* @brief Returns the bitmask representing all active errors on the motor.
	* 
	* @return uint16_t  The bitmask containing all active errors present on the motor.
	* @note		To check for a specific error, check whether the bit which corresponds with
	*			the error value is currently set, we recommend using a bitwise AND with the
	*			value of the error of interest. See the ORCA Reference Manual, section 
	*			'Errors->Active and Latched Error Registers' for details on the error types.
	*/
	OrcaResult<uint16_t> get_errors();

	/**
	* @brief	Write to the ORCA control register to change the mode of operation of the motor.
	*			Also changes what type of command stream will be sent during high speed streaming.
	*/
	OrcaError set_mode(MotorMode orca_mode);

	/**
	* @brief Requests from the motor what mode of operation the motor is currently in.
	* 
	* @return MotorMode  The mode of operation that the ORCA is currently in.
	*/
	OrcaResult<MotorMode> get_mode();

	/**
	 * @brief	Clears all errors that are currently active on the motor.
	 * @note	If the condition that caused the error(s) is still present, those
	 *			errors will immediately appear again.
	 */
	OrcaError clear_errors();

#pragma region GENERIC_MODBUS_COMMUNICATION

	/**
	 * @brief Reads a double-wide (32-bit) register from the motor. 
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
	 * 
	 * @note The maximum read size is 125 registers.
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
	 * @brief Writes a double-wide (32-bit) register to the motor.
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
	 * @param write_data An array containing the values to be written
	 * @overload Actuator::write_multiple_registers_blocking(uint16_t reg_start_address, uint8_t num_registers, uint16_t* write_data, MessagePriority priority = MessagePriority::important))

	 * 
	 * @note The maximum write size is 123 registers.
	 */
	OrcaError write_multiple_registers_blocking(uint16_t reg_start_address, uint8_t num_registers, uint16_t* write_data, MessagePriority priority = MessagePriority::important);
	/**
	 * @brief Writes multiple register values to the motor.
	 *
	 * @param reg_start_address The starting register address to be written to
	 * @param write_data A vector containing the values to be written
	 * @overload Actuator::write_multiple_registers_blocking(uint16_t reg_start_address, std::vector<uint16_t> write_data, MessagePriority priority = MessagePriority::important)
	 * 
	 * @note The maximum write size is 123 registers.
	 */
	OrcaError write_multiple_registers_blocking(uint16_t reg_start_address, std::vector<uint16_t> write_data, MessagePriority priority = MessagePriority::important);
		
	/**
	 * @brief Simultaneously reads a set of values from the motor and writes a set of values to the motor.
	 *
	 * @param read_starting_address The starting register address to be read from
	 * @param read_num_registers The amount of registers to read
	 * @param write_starting_address The starting register address to be written to
	 * @param write_num_registers The amount of registers to write
	 * @param write_data An array containing the values to be written
	 * @overload Actuator::read_write_multiple_registers_blocking(uint16_t read_starting_address, uint8_t read_num_registers, uint16_t write_starting_address, uint8_t write_num_registers, uint16_t* write_data, MessagePriority priority = MessagePriority::important)
	 * 
	 * @note The maximum write size is 121 registers, and maximum read size is 125 registers.
	 */
	OrcaResult<std::vector<uint16_t>> read_write_multiple_registers_blocking(
		uint16_t read_starting_address, uint8_t read_num_registers,
		uint16_t write_starting_address, uint8_t write_num_registers,
		uint16_t* write_data,
		MessagePriority priority = MessagePriority::important);

	/**
	 * @brief Simultaneously reads a set of values from the motor and writes a set of values to the motor.
	 *
	 * @param read_starting_address The starting register address to be read from
	 * @param read_num_registers The amount of registers to read
	 * @param write_starting_address The starting register address to be written to
	 * @param write_data A vector containing the values to be written
	 * @overload Actuator::read_write_multiple_registers_blocking(uint16_t read_starting_address, uint8_t read_num_registers, uint16_t write_starting_address, std::vector<uint16_t> write_data, MessagePriority priority = MessagePriority::important)
	 * 
	 * @note The maximum write size is 121 registers, and maximum read size is 125 registers.
	 */
	OrcaResult<std::vector<uint16_t>> read_write_multiple_registers_blocking(
		uint16_t read_starting_address, uint8_t read_num_registers,
		uint16_t write_starting_address, std::vector<uint16_t> write_data,
		MessagePriority priority = MessagePriority::important);

	/**
	 *	Begins logging all serial communication between this application/object
	 *			and the motor that this application is talking to.
	 *	@param	log_name	The name of the file to be written to. Assumes relative path
							to the location of the built executable file.
	 *  @overload	Actuator::begin_serial_logging(const std::string& log_name)
	 */
	OrcaError begin_serial_logging(const std::string& log_name);
	/** 
	 *	Begins logging using a custom log implementation.
	 *	@param	log_name	The name of the file to be written to. Assumes relative path
							to the location of the built executable file.
	 *	@param	log	A pointer to a custom implementation of the LogInterface. In case custom logging
	 *				behaviour is desired.
	 *  @overload	Actuator::begin_serial_logging(const std::string& log_name, std::shared_ptr<LogInterface> log)
	 */
	OrcaError begin_serial_logging(const std::string& log_name, std::shared_ptr<LogInterface> log);

#pragma endregion

#pragma region STREAMING

	/**
	 *	@brief	The member variable in which responses to command stream messages are stored.
	 */
	StreamData stream_cache;

	/**
	 *	@brief	The normal run loop for motor communication. Checks for incoming serial data and
	 *			sends queued serial data. If you are communicating with your motor asynchronously, you must
	 *			call this function in a regular loop. If you are using a high speed stream, and if
	 *			there are no queued messages, injects stream commands according to the motor mode
	 *			from the most recent call to set_mode().
	 */
	void run();

	/**
	 * @brief	Enables command streaming with the ORCA. 
	 *		
	 *		Command streaming is the main form of asynchronous communication with the ORCA. Command
			streaming is also required for certain modes of communication, including Position mode, 
			Force mode, and Haptic mode. When command streaming is enabled, this object	will automatically 
			inject command stream messages when the run() function is called, and this object isn't 
			currently waiting on an active message. When command stream messages complete, this object
			populates the returned data from the motor into the stream_cache public StreamData struct member. 
			See the ORCA Series Modbus User Guide, section ORCA-specific Function Codes for details on 
			command streaming.
	 *	
	*/
	void enable_stream();

	/**
	 * @brief	Disables command streaming with the ORCA
	*/
	void disable_stream();

	/**
	* @brief Set/adjust the force that the motor is exerting when in motor_command stream mode
	*
	* @param force force, in milli-Newtons
	*/
	void set_streamed_force_mN(int32_t force);

	/**
	* @brief Set/adjust the position that the motor is aiming for when in motor command stream mode
	*
	* @param position position, in micrometers
	*/
	void set_streamed_position_um(int32_t position);

	/**
	* @brief Update which haptic effects will be set through the motor command frame
	*
	* @param effects The bitmap describing which haptic effects should be enabled and disabled
	*/
	void update_haptic_stream_effects(uint16_t effects);

#pragma endregion

#pragma region UNCOMMON_MISC_DATA

	/**
	* @brief Returns the amount of power being drawn by the motor, in Watts
	* 
	* @return uint16_t - Power in Watts
	*/
	OrcaResult<uint16_t> get_power_W();

	/**
	* @brief Returns the temperature of the motor's board/microcontroller in Celcius
	* 
	* @return uint16_t - Temperature of the board/microcontroller in Celcius
	*/
	OrcaResult<int16_t> get_temperature_C();

	/**
	* @brief Returns the temperature of the motor's coil in Celcius
	* 
	* @return uint16_t - Temperature of the coil in Celcius
	*/
	OrcaResult<int16_t> get_coil_temperature_C();

	/**
	* @brief Returns the amount of voltage the motor is recieving, in milli-Volts. 
	* 
	* @return uint16_t - Voltage in milli-Voltage 
	*/
	OrcaResult<uint16_t> get_voltage_mV();

	/**
	 * @brief Sets the zero position of the motor to be the currently sensed position 
	 */
	OrcaError zero_position();

	/**
	 * @brief Copies the register for latched errors from the ORCA memory map into the local memory map 
	 * 
	 *		Latched errors are errors that were found by the motor, but are no longer active (not happening anymore)
	 */
	OrcaResult<uint16_t> get_latched_errors();

	/**
	 *	@brief	Returns the time, in microseconds, since the last successful message with the motor has completed.
	 *	@return int64_t	Time in microseconds since the last message completed successfully.
	 */
	int64_t time_since_last_response_microseconds();

#pragma endregion

#pragma region TUNING_AND_SAFETY

	/**
	 * @brief Set the maximum force that the motor allows
	 * 
	 * @param max_force force in milli-Newtons
	 */
	OrcaError set_max_force(int32_t max_force);

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
	 * @brief	Sets the PID controller tuning values on the motor for the position controller.
	 * 
	 * @param  pgain proportional gain
	 * @param  igain integral gain
	 * @param  dvgain derivative gain with respect to velocity
	 * @param  sat maximum force (safety value)
	 * @param  degain derivative gain with respect to error
	 * @note	The position controller's PID tuning affects the behaviour of the motor in 
	 *			both position control mode and kinematic control mode. Please refer to the 
	 *			ORCA Series Reference Manual, section Controllers->Position Controller
	 *			for details.
	 */
	void tune_position_controller(uint16_t pgain, uint16_t igain, uint16_t dvgain, uint32_t sat, uint16_t degain=0);

#pragma endregion

#pragma region KINEMATICS

	/**
	 * @brief	Sets the parameters of a kinematic motion.
	 * @param	ID	The ID of the motion.
	 * @param	position  The target position to reach
	 * @param	time  The desired time, in milliseconds, to complete the motion
	 * @param	delay	 The desired delay, in milliseconds, between this motion and the chained motion to execute after this
	 * @param	type  0 = minimize power, 1 = maximize smoothness
	 * @param	auto_next  Should this motion to another motion after completing?
	 * @param	next_id  The motion that should be executed next, in the case that chain is set to true
	 * @note	Please refer to the ORCA Series Reference Manual, section Controllers->Kinematic Controller
	 *			for details.
	 */
	OrcaError set_kinematic_motion(int8_t ID,int32_t position, int32_t time, int16_t delay, int8_t type, int8_t auto_next, int8_t next_id = -1);

	/**
	 * @brief Trigger to start a kinematic motion, this will also run any chained motions
	 * @param	ID  Identification of the motion to be triggered
	 * @note	This function will only have a result if the motor is currently in kinematic mode.
 	 *			Please refer to the ORCA Series Reference Manual, section Controllers->Kinematic Controller
 	 *			for details.
	 */
	OrcaError trigger_kinematic_motion(int8_t ID);

#pragma endregion

#pragma region HAPTICS

	/**
	 *	@brief	An enum representing the values of different bits for constructing an active
	 *			haptic effects bitmask.
	 *  @deprecated  Use ORCAReg::HAPTIC_STATUS_Values definitions
	 */
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
	 *  @brief	Sets each haptic effect to enabled or disabled according to the input bits.
	 *	@param	effects  The bitmask representing which haptic effects should be enabled. Can be any combination
				of haptic effects. See ORCAReg::HAPTIC_STATUS_Values for options.
	 *	@note	Please refer to the ORCA Series Reference Manual, section Controllers->Haptic Controller
	 *			for details.
	*/
	OrcaError enable_haptic_effects(uint16_t effects);

	/**
	 *	@brief	Enum representing valid options for different Spring Coupling settings
	 *  @deprecated  Use ORCAReg::Sn_COUPLING_Values definitions
	 */
	enum SpringCoupling
	{
		both, //!< Apply spring force in both directions
		positive, //!< Apply spring force in only the positive direction
		negative //!< Apply spring force in only the negative direction
	};

	/** 
	 *	@brief	Configures a spring effect with the given parameters.
	 *	@param	spring_id	The ID of the spring effect to be configured.
	 *	@param	gain	The amount of gain (force per distance from spring center) for this spring effect.
	 *	@param	center	The center of the spring effect. The position where no spring force will be applied.
	 *	@param	dead_zone	The radius of the 'dead zone' for the spring. For any position within the radius
	 *						of the dead zone from the spring center, no spring force will be applied.
	 *	@param	saturation	The maximum force that can be applied by the spring
	 *	@param	coupling	The directions from the center in which the spring force applies. See ORCAReg::Sn_COUPLING_Values for options.
	 *	@note	Please refer to the ORCA Series Reference Manual, section Controllers->Haptic Controller
	 *			for details.
	 */
	OrcaError set_spring_effect(
		uint8_t spring_id, 
		uint16_t gain, 
		int32_t center, 
		uint16_t dead_zone = 0, 
		uint16_t saturation = 0,
		uint16_t coupling = ORCAReg::Sn_COUPLING_Values::BOTH);

	/**
	 *	@brief	Enum representing options for different Oscillator Types
	 *  @deprecated  Use ORCAReg::On_TYPE_Values definitions
	 */
	enum OscillatorType
	{
		Pulse = 0, //!< Create a pulse oscillation effect. Requires setting the duty cycle parameter
		Sine, //!< Create a sine wave oscillation
		Triangle, //!< Create a triangle shape oscillation
		Saw //!< Create a saw shape oscillation
	};

	/**
	 *	@brief	Configures the parameters of an oscillation effect with the given parameters.
	 *	@param	osc_id	The oscillation effect's ID number.
	 *	@param	amplitude	The amplitude of the oscillation effect.
	 *	@param	frequency_dhz	The frequency, in decihertz, of the oscillation effect.
	 *	@param	duty	The duty-cycle of the oscillation effect. Only relevant for pulse type effects.
	 *	@param	type	The type of oscillation effect to create. See ORCAReg::On_TYPE_Values for options.
	 *	@note	Please refer to the ORCA Series Reference Manual, section Controllers->Haptic Controller
	 *			for details.
	 */
	OrcaError set_osc_effect(uint8_t osc_id, uint16_t amplitude, uint16_t frequency_dhz, uint16_t duty, uint16_t type);

	/**
	*	@brief Sets the damping value in Haptic Mode
	*	@param damping	The damping gain (4*N*s/mm)
	 *	@note	Please refer to the ORCA Series Reference Manual, section Controllers->Haptic Controller
	 *			for details.
	*/
	OrcaError set_damper(uint16_t damping);

	/**
	*	@brief Sets the inertia value in Haptic Mode
	*	@param inertia	The inertia gain (64*N*s^2/mm)
	 *	@note	Please refer to the ORCA Series Reference Manual, section Controllers->Haptic Controller
	 *			for details.
	*/
	OrcaError set_inertia(uint16_t inertia);

	/**
	*	@brief Sets the constant force value in Haptic Mode
	*	@param force	(mN)
	 *	@note	Please refer to the ORCA Series Reference Manual, section Controllers->Haptic Controller
	 *			for details.
	*/
	OrcaError set_constant_force(int32_t force);

	/**
	*	@brief Sets the constant force filter value in Haptic Mode
	*	@param force_filter	Amount of filtering on constant force inputs
	 *	@note	Please refer to the ORCA Series Reference Manual, section Controllers->Haptic Controller
	 *			for details.
	*/
	OrcaError set_constant_force_filter(uint16_t force_filter);

#pragma endregion

#pragma region MOTOR_ID_AND_VERSIONING

	/**
	* @brief Returns the actuator serial number
	*
	* @return int32_t - actuator serial number
	*/
	OrcaResult<int32_t> get_serial_number();

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

#pragma endregion

private:
	OrcaError message_error{false};
	std::vector<uint16_t> message_data{};
	
	OrcaStream stream;

	const uint8_t modbus_server_address;
	int64_t _time_since_last_response_microseconds;

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

	/**
	 *	@brief	If called with parameter true, pauses automatic injection of stream
	 *			commands during calls to run()
	 */
	void set_stream_paused(bool paused);

	bool stream_paused = false;
};

}

#endif