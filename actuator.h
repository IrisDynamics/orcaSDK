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
#include <string>


/**
   @class Actuator
   @brief Object that abstracts the communications between the client and a Orca motor server.
 */
class Actuator {

	std::shared_ptr<SerialInterface> serial_interface;

public:
	ModbusClient modbus_client;

#if defined(WINDOWS)
	void set_new_comport(int _comport);

	void disable_comport();
#endif

public:
	Actuator(
		int uart_channel
	);

	Actuator(
		std::shared_ptr<SerialInterface> serial_interface,
		int uart_channel
	);

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

	/**
	* @brief Returns the total amount of force being sensed by the motor
	*
	* @return uint32_t - force in milli-Newtons
	*/
	int32_t get_force_mN();

	/**
	* @brief Returns the position of the shaft in the motor (distance from the zero position) in micrometers.
	*
	* @return uint32_t - position in micrometers
	*/
	int32_t get_position_um();

	/**
	*@brief Get to a good handshake init state and set up the device driver with the default baud rate
	*/
	void init();

	void run();

	/**
	 * @brief process all commands in modbus queue
	 */
	void flush();

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
	* @brief Returns the UART channel number in use
	* 
	* @return int, channel number
	*/
	int channel_number();

	/**
	*@brief get the motor's mode of operations as currently updated by the local memory map
	*/
	uint16_t get_mode_of_operation();

	/**
	* @brief Returns the sum of all error messages being sent by the motor
	* 
	* @return uint16_t - sum or all active error codes
	*/
	uint16_t get_errors();

	/**
	 * @brief clear all errors stored on the motor
	 * note: errors that are still found will appear again
	 */
	void clear_errors();

	/**
	* @brief Returns the amount of power being drawn by the motor, in Watts
	* 
	* @return uint16_t - power in Watts
	*/
	uint16_t get_power_W();

	/**
	* @brief Returns the temperature of the motor in Celcius
	* 
	* @return uint8_t - temperature in Celcius
	*/
	uint16_t get_temperature_C();

	/**
	* @brief Returns the amount of voltage the motor is recieving, in milli-Volts. 
	* 
	* @return uint16_t - voltage in milli-Voltage 
	*/
	uint16_t get_voltage_mV();

	/**
	* @brief Returns the actuator serial number
	* 
	* @return uint32_t - actuator serial number
	*/
	uint32_t get_serial_number();

	/**
	* @brief Return the firmware major version
	* 
	* @return uint16_t - firmware major version
	*/
	uint16_t get_major_version();

	/**
	* @brief Return the firmware release state (minor version)
	* 
	* @return uint16_t - firmware release state
	*/
	uint16_t get_release_state();

	/**
	* @brief Return the firmware revision number
	* 
	* @return uint16_t - firmware revision number
	*/
	uint16_t get_revision_number();
	
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
	bool version_is_at_least(uint8_t version, uint8_t release_state, uint8_t revision_number);

	/**
	 * @brief Set the zero position of the motor to be the current position 
	 */
	void zero_position();

	/**
	 * @brief Copies the register for latched errors from the orca memory map into the local memory map 
	 * Latched errors are errors that were found by the motor, but are no longer active (not happening anymore)
	 */
	void get_latched_errors();

	/**
	 * @brief Set the maximum force that the motor allows
	 * 
	 * @param max_force force in milli-Newtons
	 */
	void set_max_force(s32 max_force);

	/**
	 * @brief Set the maximum temperature that the motor allows
	 * 
	 * @param max_temp temperature in Celcius
	 */
	void set_max_temp(uint16_t max_temp);

	/**
	 * @brief Set the maximum power that the motor allows
	 * 
	 * @param max_power power in Watts
	 */
	void set_max_power(uint16_t max_power);

	/**
	 * @brief Sets the fade period when changing position controller tune in ms
	 * 
	 * @param t_in_ms time period in milliseconds
	*/
	void set_pctrl_tune_softstart(uint16_t t_in_ms);

	/**
	 * @brief Sets the motion damping gain value used when communications are interrupted.
	 * 
	 * @param max_safety_damping damping value
	 */
	void set_safety_damping(uint16_t max_safety_damping);

	/**
	 * @brief Sets the PID tuning values on the motor in non-scheduling mode. Disabled the gain scheduling in motors that support it
	 * 
	 * @param pgain proportional gain
	 * @param igain integral gain
	 * @param dgain derivative gain
	 * @param sat maximum force (safety value)
	 */
	void tune_position_controller(uint16_t pgain, uint16_t igain, uint16_t dvgain, uint32_t sat, uint16_t degain=0);

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
	void set_kinematic_motion(int8_t ID,int32_t position, int32_t time, int16_t delay, int8_t type, int8_t auto_next, int8_t next_id = -1);

	/**
	* @brief Use the software trigger to start a kinematic motion, this will also run any chained motions
	* @ID Identification of the motion to be triggered
	*/
	void trigger_kinematic_motion(int8_t ID);

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
	* @brief Enable or disabled desired haptic effects.
	*/
	void enable_haptic_effects(uint16_t effects);

	/** @brief update the spring effect in a single function
	*/
	void set_spring_effect(u8 spring_id, u16 gain, u32 center, u16 dead_zone = 0, u16 saturation = 0, u8 coupling = 0);

	/**
	*/
	void set_osc_effect(u8 osc_id, u16 amplitude, u16 frequency_dhz, u16 duty, u16 type);

#pragma endregion

	/**
	 * @brief Request for a specific register in the local copy to be updated from the motor's memory map
	 * 
	 * @param reg_address register address
	 */
	void read_register(uint16_t reg_address, MessagePriority priority = MessagePriority::important);
	
	/**
	 * @brief Request for multiple sequential registers in the local copy to be updated from the motor's memory map
	 *
	 * @param reg_address register address from the orca's memory map
	 * @param num_registers number of sequential registers to read
	 */
	void read_registers(uint16_t reg_address, uint8_t num_registers, MessagePriority priority = MessagePriority::important);

	/**
	 * @brief Request for a specific register in the motor's memory map to be updated with a given value.
	 * 
	 * @param reg_address register address
	 * @param reg_data data to be added to the register
	 */
	void write_register(uint16_t reg_address, uint16_t reg_data, MessagePriority priority = MessagePriority::important);

	/**
	 * @brief Request for multiple registers in the motor's memory map to be updated with a given value.
	 *
	 * @param reg_address register address
	 * * @param num_registers number of sequential registers to write
	 * @param reg_data pointer to an array of data to be added to the registers
	 */
	void write_registers(uint16_t reg_address, uint8_t num_registers, uint8_t* reg_data, MessagePriority priority = MessagePriority::important);

	void write_registers(uint16_t reg_address, uint8_t num_registers, uint16_t* reg_data, MessagePriority priority = MessagePriority::important);

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
		uint16_t read_starting_address, uint8_t read_num_registers,
		uint16_t write_starting_address, uint8_t write_num_registers,
		uint8_t* write_data,
		MessagePriority priority = MessagePriority::important);

	/**
	* @brief Return the contents of the given register from the controller's copy of the motor's memory map. 
	* 
	* @param offset the register that will be read
	* @return uint16_t - register contents
	*/
	uint16_t get_orca_reg_content(uint16_t offset);

	void begin_serial_logging(const std::string& log_name);
	void begin_serial_logging(const std::string& log_name, std::shared_ptr<LogInterface> log);

private:
	std::array<uint16_t, ORCA_REG_SIZE> orca_reg_contents{};

	void handle_transaction_response(Transaction response);

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
	* @brief Write to the orca control register to change the mode of operation of the motor
	* note some modes require a constant stream to stay in that mode (eg. force, position)
	*/
	void set_mode(MotorMode orca_mode);

	/**
	* @brief the communication mode determines which commands are sent by enqueue_motor_frame
	* *
	* @return CommunicationMode
	*/
	MotorMode get_mode();

	/**
	* @brief Set the type of high speed stream to be sent on run out once handshake is complete
	*/
	void set_stream_mode(StreamMode mode);

	/**	
	* @brief Get the current stream type to be sent on run out once handshake is complete
	*/
	StreamMode get_stream_mode();

	/**
	* @brief This function can be continuously called and will update the values being sent when in motor write stream mode
	*/
	void update_write_stream(uint8_t width, uint16_t register_address, uint32_t register_value);

	/**
	* @brief This function can be continuously called and will update the values being sent when in motor read stream mode
	*/
	void update_read_stream(uint8_t width, uint16_t register_address);

	/**
	* @brief Set the maximum time required between calls to set_force or set_position, in force or position mode respectively, before timing out and returning to sleep mode. 
	* 
	* @param timout_us time in microseconds
	*/
	void set_stream_timeout(uint64_t timeout_us);

	/**
	 * @brief Error check and apply the handshake/connection configuration parameters passed in the ConnectionConfig struct
	 *
	 * @param config ConnectionConfig object
	 * @return 0 if one of the parameters was invalid and default values were used, 1 otherwise
	*/
	int set_connection_config(Actuator::ConnectionConfig config);

	/**
	 * @brief Determine if communication with a server is enabled or not
	 *
	 * @return boolean - the enabled status (true if enabled, false otherwise)
	*/
	bool is_enabled();

	/**
	 * @brief Enable communication with a server device. Allows handshake sequence to begin, enables transceiver hardware
	*/
	void enable();

	/**
	 * @brief Disable communication with a server device. Moves into disconnecting state where transceiver hardware will be disabled
	*/
	void disable();

	/**
		* @brief Determine whether a server has successfully connected with this client
		* @return true if the server is in the connected state, false otherwise
	*/
	bool is_connected();

	/**
		* @brief Reset variables and move into the disconnected state
	*/
	void disconnect();
	enum class ConnectionStatus {
		disconnected,	// reset state
		discovery, 	// sending discovery pings, negotiating baud rate and delay
		synchronization,
		negotiation,
		connected,	// streaming commands to the server
	};

	ConnectionStatus connection_state = ConnectionStatus::disconnected;

	void initiate_handshake();

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
	void modbus_handshake(Transaction response);
private:

	StreamMode stream_mode = MotorCommand;
	MotorMode comms_mode = SleepMode;

	uint64_t stream_timeout_start = 0;
	uint64_t stream_timeout_cycles = 100000;

	// Used to hold the last commanded force and position commands from the user of this object
	int32_t force_command = 0;
	int32_t position_command = 0;
	//Used to hold the last data to stream in motor write and read streams
	uint32_t motor_write_data = 0;
	uint16_t motor_write_addr = 0;
	uint8_t motor_write_width = 1;
	uint16_t motor_read_addr = 0;
	uint8_t motor_read_width = 1;

	bool stream_paused = false;

	// These counters are used to find the success and failure rate of the comms
	int32_t success_msg_counter = 0, failed_msg_counter = 0;

	/**
	 * @brief Requests the actuator synchronize its memory map with the controller
	 */
	void synchronize_memory_map();

	/**
	 * @brief Resets the memory map array to zeros
	 */
	void desynchronize_memory_map();

	static constexpr int kinematic_command = 32;
	static constexpr int haptic_command = 34;
	void motor_stream_command();

	void motor_stream_read();

	void motor_stream_write();

	/**
	 * @brief enqueue a motor message if the queue is empty
	 */
	void enqueue_motor_frame();

	/**
	 * @brief Determine the length of the request for an application specific function code
	 *
	 * @param fn_code application specific function code
	 * @return int - length of request
	 */
	int get_app_reception_length(uint8_t fn_code);

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
	int motor_command_fn(uint8_t device_address, uint8_t command_code, int32_t register_value);

	int motor_read_fn(uint8_t device_address, uint8_t width, uint16_t register_address);

	int motor_write_fn(uint8_t device_address, uint8_t width, uint16_t register_address, uint32_t register_value);

	ConnectionConfig connection_config;
	/**
	 * @brief Description of the possible connection states between the client and a server
	 *        Main state machine can be found in IrisClientApplication.
	 *        ModbusClient only responsible for moving to the 'disconnecting' state upon missed responses or errors
	*/

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
	int enqueue_change_connection_status_fn(uint8_t device_address, bool connect, uint32_t baud_rate_bps, uint16_t delay_us);

	void handle_stream();

private:

	int num_discovery_pings_received = 0;

	/////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////  Pause Timer  ///
	///////////////////////////////////////////////////////////////////////////

	bool is_paused = false;
	uint64_t  pause_timer_start = 0;
	static constexpr uint64_t pause_time_cycles{ DEFAULT_CONNECTION_PAUSE_uS };

	/**
	* @brief Start the pause timer. This can be done by saving the system time when the timer was started. Should not use interrupt timer
	*/
	void start_pause_timer();
	/**
	* @brief Check the progress of the 200 millisecond interval (pause timer)
	* @return The remaining time in the interval, 0 if the interval is not set or has finished
	*/
	bool has_pause_timer_expired();

	/**
	* @brief Format a Transaction to check the communication with a certain server
	* @return 1 if the request was added to the queue, 0 if the queue was full
	*
	*/
	int enqueue_ping_msg();

#pragma endregion


	////////////////////////////////////////////////////////////////////
	
public:
	[[deprecated("Requests initialization of now unused parameters")]]
	Actuator(
		int uart_channel,
		const char* name,
		int
	) :
		Actuator(uart_channel)
	{}
};

#endif