#pragma once

#include "modbus_client.h"
#include "orca_stream_config.h"
#include "orca_modes.h"

class Actuator;

class OrcaStream
{
public:
	/**
	*@brief Sets the type of command that will be sent on high speed stream (ie when enable() has been used, this sets the type of message sent from enqueue motor frame)
	*/
	typedef enum {
		MotorCommand,
		MotorRead,
		MotorWrite
	} StreamMode;

	OrcaStream(Actuator* motor, ModbusClient& modbus_client, uint8_t modbus_server_address);

	/**
	 * @brief Determine if communication with a server is enabled or not
	 *
	 * @return boolean - the enabled status (true if enabled, false otherwise)
	*/
	bool is_enabled();

	void enable();

	void disable();

	/**
		* @brief Reset variables and move into the disconnected state
	*/
	void disconnect();

	bool is_connected();
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

	void update_stream_state(Transaction response);
	void handle_stream();
	void set_connection_config(ConnectionConfig config);

	/**
	 * @brief enqueue a motor message if the queue is empty
	 */
	void enqueue_motor_frame();
	/**
	* @brief Set the type of high speed stream to be sent on run out once handshake is complete
	*/
	void set_stream_mode(OrcaStream::StreamMode mode);

	/**
	 * @brief Format a change_connection_status request, user-defined function code 65, and add the request to the buffer queue
	 * @param device_address
	 * @param connect true to connect the server, false to disconnect
	 * @param baud_rate_bps
	 * @param delay_us
	*/
	int enqueue_change_connection_status_fn(uint8_t device_address, bool connect, uint32_t baud_rate_bps, uint16_t delay_us);

	void update_motor_mode(MotorMode mode);

	void update_write_stream(uint8_t width, uint16_t register_address, uint32_t register_value);

	void update_read_stream(uint8_t width, uint16_t register_address);

	void set_force_mN(int32_t force);

	void set_position_um(int32_t position);

	void set_haptic_effects(uint16_t effects);

private:
	Actuator* motor;
	ModbusClient& modbus_client;

	const uint8_t modbus_server_address;

	bool enabled = false;

	MotorMode comms_mode = SleepMode;

	// Used to hold the last commanded force and position commands from the user of this object
	int32_t force_command = 0;
	int32_t position_command = 0;
	uint16_t haptic_command_effects = 0;
	//Used to hold the last data to stream in motor write and read streams
	uint32_t motor_write_data = 0;
	uint16_t motor_write_addr = 0;
	uint8_t motor_write_width = 1;
	uint16_t motor_read_addr = 0;
	uint8_t motor_read_width = 1;

	int num_discovery_pings_received = 0;

	/////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////  Pause Timer  ///
	///////////////////////////////////////////////////////////////////////////

	bool is_paused = false;
	int64_t  pause_timer_start = 0;
	static constexpr int64_t pause_time_cycles{ DEFAULT_CONNECTION_PAUSE_uS };


	/**
	 * @brief Description of the possible connection states between the client and a server
	 *        Main state machine can be found in IrisClientApplication.
	 *        ModbusClient only responsible for moving to the 'disconnecting' state upon missed responses or errors
	*/
	enum class ConnectionStatus {
		disconnected,	// reset state
		discovery, 	// sending discovery pings, negotiating baud rate and delay
		synchronization,
		negotiation,
		connected,	// streaming commands to the server
	};

	ConnectionStatus connection_state = ConnectionStatus::disconnected;
	// This is used to determine when a connection has terminated and the ConnectionStatus should change to disconnecting
	int cur_consec_failed_msgs = 0;
	// These counters are used to find the success and failure rate of the comms
	int32_t success_msg_counter = 0, failed_msg_counter = 0;

	ConnectionConfig connection_config;

	StreamMode stream_mode = MotorCommand;

	static constexpr int kinematic_command_code = 32;
	static constexpr int haptic_command_code = 34;

	void motor_stream_command();

	void motor_stream_read();

	void motor_stream_write();

	/**
	  @brief Format a motor command request, function code 0x64, and add the request to the buffer queue

	  @param device_address Server device address
	  @param command_code command code to specify command mode (sleep, force, position etc.)
	  @param register_value The value to write to the register
	 */
	int motor_command_fn(uint8_t device_address, uint8_t command_code, int32_t register_value);

	int motor_read_fn(uint8_t device_address, uint8_t width, uint16_t register_address);

	int motor_write_fn(uint8_t device_address, uint8_t width, uint16_t register_address, uint32_t register_value);

	/**
	 * @brief Determine the length of the request for an application specific function code
	 *
	 * @param fn_code application specific function code
	 * @return int - length of request
	 */
	int get_app_reception_length(uint8_t fn_code);

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
	int enqueue_ping_msg(uint8_t device_address);
};