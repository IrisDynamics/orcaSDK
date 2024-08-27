#include "pch.h"
#include "modbus/helpers/TestModbusClient.h"
#include "actuator.h"
#include <memory>
#include "helpers/modbus_helpers.h"

#undef WINDOWS

class ActuatorTests : public testing::Test
{
protected:
	ActuatorTests() :
		serial_interface(std::make_shared<TestModbusClient>()),
		motor(serial_interface, -1, "Hello")
	{}

	std::shared_ptr<TestModbusClient> serial_interface;
	Actuator motor;
};

TEST_F(ActuatorTests, ReadingFromMemoryMapAndThenReceivingUpdatesLocalMemoryMap)
{
	motor.read_registers(SHAFT_POS_UM, 2);
	motor.run_out();
	
	std::vector<char> expected_sent_buffer{
		'\x1', '\x3', '\x1', '\x56', '\0', '\x2', '\x25', '\xe7'
	};

	EXPECT_EQ(expected_sent_buffer, serial_interface->sendBuffer);

	std::deque<char> next_input_message = std::deque<char>{
		//						 low  reg(2)  high reg(65536)
			'\x1', '\x3', '\x4', '\0', '\x2', '\0', '\x1'
	};
	ModbusTesting::CalculateAndAppendCRC(next_input_message);
	serial_interface->consume_new_message(next_input_message);

	motor.run_in();

	EXPECT_EQ(65538, motor.get_position_um());
}

// This test walks through the 'happy path' of the modbus high speed stream handshake
//  mostly to protect against regressions. Additional tests should be added to handle
//  bad paths through the algorithm.
TEST_F(ActuatorTests, ModbusHighSpeedStreamHandshakeHappyPathIntegrationTest)
{
	//-----Disconnected-----
	motor.enable();

	//Trigger beginning of discovery
	motor.run_out();

	std::vector<char> ping_message{
		{ '\x1', '\b', '\0', '\0', '\x80', '\x1a' }
	};
	EXPECT_EQ(ping_message, serial_interface->sendBuffer);

	//-----Discovery-----
	// Involves sending three ping messages and hearing three echo responses
	// After performing this back and forth, enqueues messages to synchronize
	// the local memory map, and enters beginning of synchronization
	std::deque<char> ping_echo;
	for (char c : ping_message)
	{
		ping_echo.push_back(c);
	}
	serial_interface->sendBuffer.clear();
	serial_interface->consume_new_message(ping_echo);
	motor.run_in();
	serial_interface->pass_time(2001);
	motor.run_out();
	serial_interface->sendBuffer.clear();
	serial_interface->consume_new_message(ping_echo);
	motor.run_in();
	serial_interface->pass_time(2001);
	motor.run_out();
	serial_interface->sendBuffer.clear();
	serial_interface->consume_new_message(ping_echo);
	motor.run_in();
	serial_interface->pass_time(2001);
	motor.run_out();

	//-----Synchronization-----
	// Sends messages asking for memory reads for different memory regions
	// Completes when each read returns with the correct shape and parameters
	// and Actuator::run_in() consumes all 3 messages
	std::vector<char> first_sync_message{ '\x1', '\x03', '\x01', '\x90', '\0', '\x13', '\x5', '\xd6' };
	EXPECT_EQ(first_sync_message, serial_interface->sendBuffer);

	std::deque<char> first_sync_response{
		'\x1', '\x3', '\x13'
	};
	for (int i = 0; i < 0x13 * 2; i++) // Push 19 (0x13) register values into the receive buffer
	{
		first_sync_response.push_back('\0');
	}
	ModbusTesting::CalculateAndAppendCRC(first_sync_response);
	serial_interface->consume_new_message(first_sync_response);

	serial_interface->sendBuffer.clear();
	motor.run_in();
	serial_interface->pass_time(2001);
	motor.run_out();

	std::vector<char> second_sync_message{ '\x1', '\x03', '\x01', '\xb0', '\0', '\x5', '\x85', '\xd2' };
	EXPECT_EQ(second_sync_message, serial_interface->sendBuffer);

	std::deque<char> second_sync_response{
		'\x1', '\x3', '\x5'
	};
	for (int i = 0; i < 0x5 * 2; i++) // Push 5 register values into the receive buffer
	{
		second_sync_response.push_back('\0');
	}
	ModbusTesting::CalculateAndAppendCRC(second_sync_response);
	serial_interface->consume_new_message(second_sync_response);

	serial_interface->sendBuffer.clear();
	motor.run_in();
	serial_interface->pass_time(2001);
	motor.run_out();

	std::vector<char> third_sync_message{ '\x1', '\x03', '\0', '\x80', '\0', '\x19', '\x85', '\xe8' };
	EXPECT_EQ(third_sync_message, serial_interface->sendBuffer);

	std::deque<char> third_sync_response{
		'\x1', '\x3', '\x19'
	};
	for (int i = 0; i < 0x19 * 2; i++) // Push 24 (0x18) register values into the receive buffer
	{
		third_sync_response.push_back('\0');
	}
	ModbusTesting::CalculateAndAppendCRC(third_sync_response);
	serial_interface->consume_new_message(third_sync_response);
	
	//Consume the 3 ready response messages.
	motor.run_in();

	serial_interface->sendBuffer.clear();
	serial_interface->pass_time(2001);
	motor.run_out();

	//-----Negotiation-----
	// The motor sends exactly one stream negotiation message and 
	// expects a valid response. After receiving it, it updates
	// relevant local parameters and goes to connected state.
	std::deque<char> stream_negotiation_response{
			'\x01', //Orca address
			65, //Manage High-speed Stream message ID
			'\xff', '\0', // Echo of enable and apply parameters
			'\0', '\0', '\0', '\xff', // Target baud rate (255bps)
			'\0', '\xa0', // Target Response delay (128ms)
			'\x62', '\xdd' // CRC calculated for custom response
	};
	serial_interface->consume_new_message(stream_negotiation_response);
	motor.run_in();
	motor.run_out();

	EXPECT_TRUE(motor.is_connected());
}

TEST_F(ActuatorTests, ReadWriteMultipleRegistersSendsCorrectDataAndPopulatesLocalCacheOnReceive)
{
	uint8_t data[2] = { 0x01, 0x01 };
	motor.read_write_multiple_registers_fn(1, 97, 2, 180, 1, data);
	motor.run_out();

	std::vector<char> expected_sent_buffer{
		'\x1', '\x17', '\x0', '\x61', '\0', '\x2', '\0', '\xb4', '\x0', '\x1', '\x2', '\x1', '\x1', '\x9d', '\x24'
	};

	EXPECT_EQ(expected_sent_buffer, serial_interface->sendBuffer);

	std::deque<char> expected_response = std::deque<char>{
			'\x01', '\x17', '\x04', '\x0d', '\x9d', '\x0e', '\x3e'
	};
	ModbusTesting::CalculateAndAppendCRC(expected_response);
	serial_interface->consume_new_message(expected_response);

	motor.run_in();

	EXPECT_EQ(motor.get_orca_reg_content(97), 0xd9d);
	EXPECT_EQ(motor.get_orca_reg_content(98), 0xe3e);
}

TEST_F(ActuatorTests, QueueingMultipleReadsResultsInBothCompleting)
{
	motor.read_registers(SHAFT_POS_UM, 2);
	motor.read_registers(SHAFT_SPEED_MMPS, 2);
	motor.run_out();

	std::deque<char> first_message_response = std::deque<char>{
		//						 low  reg(2)  high reg(65536)
			'\x1', '\x3', '\x4', '\0', '\x2', '\0', '\x1'
	};
	ModbusTesting::CalculateAndAppendCRC(first_message_response);
	serial_interface->consume_new_message(first_message_response);

	motor.run_in();

	serial_interface->pass_time(2001);

	motor.run_out();

	EXPECT_EQ(65538, motor.get_position_um());

	std::deque<char> second_message_response = std::deque<char>{
		//						 low  reg(2)  high reg(65536)
			'\x1', '\x3', '\x4', '\0', '\x3', '\0', '\x2'
	};
	ModbusTesting::CalculateAndAppendCRC(second_message_response);
	serial_interface->consume_new_message(second_message_response);

	motor.run_in();

	EXPECT_EQ(3, motor.get_orca_reg_content(SHAFT_SPEED_MMPS));
	EXPECT_EQ(2, motor.get_orca_reg_content(SHAFT_SHEED_H));
}

TEST_F(ActuatorTests, MotorIncrementsCRCDiagnosticCounterOnBadCRCResponse)
{
	motor.read_register(POWER);

	motor.run_out();
	std::deque<char> receive_buffer{
		'\x1', '\x3', '\x1', '\x0', '\xff', '\x0', '\x0'
	};
	serial_interface->consume_new_message(receive_buffer);
	
	motor.run_in();

	EXPECT_EQ(1, motor.modbus_client.diagnostic_counters[crc_error_count]);
}

TEST_F(ActuatorTests, MotorIncrementsTimeoutAfterEnoughTimePassesBetweenSeeingFullMessageResponseTimeout)
{
	motor.read_register(POWER);

	motor.run_out();

	serial_interface->pass_time(50001);

	motor.run_in();

	EXPECT_EQ(1, motor.modbus_client.diagnostic_counters[return_server_no_response_count]);
}

TEST_F(ActuatorTests, MotorIncrementsIntercharTimeoutAfterEnoughTimePassesBetweenSeeingNewBytes)
{
	motor.read_register(POWER);

	motor.run_out();

	std::deque<char> new_input
	{
		'\x1'
	};
	serial_interface->consume_new_message(new_input);

	motor.run_in(); //Parse first byte
	serial_interface->pass_time(16001);

	motor.run_in(); //Timer times out from not receiving second byte

	EXPECT_EQ(1, motor.modbus_client.diagnostic_counters[unexpected_interchar]);
}

TEST_F(ActuatorTests, MotorGoesToSleepIfEnoughTimePassesBetweenForceStreamCommands)
{
	motor.enable();
	motor.connection_state = IrisClientApplication::ConnectionStatus::connected;
	motor.set_mode(Actuator::ForceMode);

	EXPECT_EQ(Actuator::ForceMode, motor.get_mode());

	serial_interface->pass_time(100001);
	motor.run_out();

	EXPECT_EQ(Actuator::SleepMode, motor.get_mode());
}

TEST_F(ActuatorTests, MotorGoesToSleepIfEnoughTimePassesBetweenPositionStreamCommands)
{
	motor.enable();
	motor.connection_state = IrisClientApplication::ConnectionStatus::connected;
	motor.set_mode(Actuator::PositionMode);

	EXPECT_EQ(Actuator::PositionMode, motor.get_mode());

	serial_interface->pass_time(100001);
	motor.run_out();

	EXPECT_EQ(Actuator::SleepMode, motor.get_mode());
}

TEST_F(ActuatorTests, WhenEnabledAndConnectedActuatorObjectAutomaticallyEnqueuesStreamCommands)
{
	motor.enable();
	motor.connection_state = IrisClientApplication::ConnectionStatus::connected;
	motor.set_mode(Actuator::PositionMode);

	motor.set_position_um(1); // This sets the stream timeout timer
	motor.run_out(); // This sends the change mode command

	std::deque<char> echo_of_mode_register_write{
		'\x1', '\x6', '\0', '\x3', '\0', '\x3', '\x39', '\xcb'
	};
	serial_interface->consume_new_message(echo_of_mode_register_write);
	motor.run_in();
	serial_interface->sendBuffer.clear();

	serial_interface->pass_time(2001);

	motor.run_out(); // This should inject a position command

	std::vector<char> position_stream_command = { '\x1', '\x64', '\x1e', '\0', '\0', '\0', '\x1', '\x6a', '\x26' };
	EXPECT_EQ(position_stream_command, serial_interface->sendBuffer);
}
