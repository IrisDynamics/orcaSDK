#include "pch.h"
#include "modbus/helpers/TestSerialInterface.h"
#include "modbus/helpers/TestLog.h"
#include "modbus/helpers/TestClock.h"
#include "actuator.h"
#include <memory>
#include "helpers/modbus_helpers.h"

#undef WINDOWS

class ActuatorTests : public testing::Test
{
protected:
	ActuatorTests() :
		serial_interface(std::make_shared<TestSerialInterface>()),
		clock(std::make_shared<TestClock>()),
		log(std::make_shared<TestLog>()),
		motor(serial_interface, clock, -1, "unimportant")
	{}

	void SetUp()
	{
		motor.begin_serial_logging("unimportant_name", log);
	}

	std::shared_ptr<TestSerialInterface> serial_interface;
	std::shared_ptr<TestClock> clock;
	std::shared_ptr<TestLog> log;
	Actuator motor;
};

TEST_F(ActuatorTests, ReadingFromMemoryMapAndThenReceivingUpdatesLocalMemoryMap)
{
	motor.read_registers(SHAFT_POS_UM, 2);
	motor.run();
	
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

	motor.run();

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
	motor.run();

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
	motor.run();
	clock->pass_time(2001);
	motor.run();
	serial_interface->sendBuffer.clear();
	serial_interface->consume_new_message(ping_echo);
	motor.run();
	clock->pass_time(2001);
	motor.run();
	serial_interface->sendBuffer.clear();
	serial_interface->consume_new_message(ping_echo);
	motor.run();
	clock->pass_time(2001);
	motor.run();

	//-----Synchronization-----
	// Sends messages asking for memory reads for different memory regions
	// Completes when each read returns with the correct shape and parameters
	// and Actuator::run() consumes all 3 messages
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
	motor.run();
	clock->pass_time(2001);
	motor.run();

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
	motor.run();
	clock->pass_time(2001);
	motor.run();

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
	motor.run();

	serial_interface->sendBuffer.clear();
	clock->pass_time(2001);
	motor.run();

	std::vector<char> output = {
		'\x01', //Orca address
		65, //Manage High-speed Stream message ID
		'\xff', '\0', // Enable and apply parameters
		'\0', '\x9', '\x89', '\x68', // Target baud rate (650000bps)
		'\0', '\x50', // Target Response delay (80ms)
		'\x25', '\x28' // CRC
	};
	ASSERT_EQ(output, serial_interface->sendBuffer);

	//-----Negotiation-----
	// The motor sends exactly one stream negotiation message and 
	// expects a valid response. After receiving it, it updates
	// relevant local parameters and goes to connected state.
	std::deque<char> stream_negotiation_response{
			output.begin(), output.end()
	};
	serial_interface->consume_new_message(stream_negotiation_response);
	motor.run();
	motor.run();

	EXPECT_TRUE(motor.is_connected());
	EXPECT_EQ(625000, serial_interface->adjusted_baud_rate);
}

TEST_F(ActuatorTests, ReadWriteMultipleRegistersSendsCorrectDataAndPopulatesLocalCacheOnReceive)
{
	uint8_t data[2] = { 0x01, 0x01 };
	motor.read_write_registers(97, 2, 180, 1, data);
	motor.run();

	std::vector<char> expected_sent_buffer{
		'\x1', '\x17', '\x0', '\x61', '\0', '\x2', '\0', '\xb4', '\x0', '\x1', '\x2', '\x1', '\x1', '\x9d', '\x24'
	};

	EXPECT_EQ(expected_sent_buffer, serial_interface->sendBuffer);

	std::deque<char> expected_response = std::deque<char>{
			'\x01', '\x17', '\x04', '\x0d', '\x9d', '\x0e', '\x3e'
	};
	ModbusTesting::CalculateAndAppendCRC(expected_response);
	serial_interface->consume_new_message(expected_response);

	motor.run();

	EXPECT_EQ(motor.get_orca_reg_content(97), 0xd9d);
	EXPECT_EQ(motor.get_orca_reg_content(98), 0xe3e);
}

TEST_F(ActuatorTests, QueueingMultipleReadsResultsInBothCompleting)
{
	motor.read_registers(SHAFT_POS_UM, 2);
	motor.read_registers(SHAFT_SPEED_MMPS, 2);
	motor.run();

	std::deque<char> first_message_response = std::deque<char>{
		//						 low  reg(2)  high reg(65536)
			'\x1', '\x3', '\x4', '\0', '\x2', '\0', '\x1'
	};
	ModbusTesting::CalculateAndAppendCRC(first_message_response);
	serial_interface->consume_new_message(first_message_response);

	motor.run();

	clock->pass_time(2001);

	motor.run();

	EXPECT_EQ(65538, motor.get_position_um());

	std::deque<char> second_message_response = std::deque<char>{
		//						 low  reg(2)  high reg(65536)
			'\x1', '\x3', '\x4', '\0', '\x3', '\0', '\x2'
	};
	ModbusTesting::CalculateAndAppendCRC(second_message_response);
	serial_interface->consume_new_message(second_message_response);

	motor.run();

	EXPECT_EQ(3, motor.get_orca_reg_content(SHAFT_SPEED_MMPS));
	EXPECT_EQ(2, motor.get_orca_reg_content(SHAFT_SHEED_H));
}

TEST_F(ActuatorTests, MotorIncrementsCRCDiagnosticCounterOnBadCRCResponse)
{
	motor.read_register(POWER);

	motor.run();
	std::deque<char> receive_buffer{
		'\x1', '\x3', '\x1', '\x0', '\xff', '\x0', '\x0'
	};
	serial_interface->consume_new_message(receive_buffer);
	
	motor.run();

	EXPECT_EQ(1, motor.modbus_client.diagnostic_counters[crc_error_count]);
}

TEST_F(ActuatorTests, MotorIncrementsTimeoutAfterEnoughTimePassesBetweenSeeingFullMessageResponseTimeout)
{
	motor.read_register(POWER);

	motor.run();

	clock->pass_time(50001);

	motor.run();

	EXPECT_EQ(1, motor.modbus_client.diagnostic_counters[return_server_no_response_count]);
}

TEST_F(ActuatorTests, MotorIncrementsIntercharTimeoutAfterEnoughTimePassesBetweenSeeingNewBytes)
{
	motor.read_register(POWER);

	motor.run();

	std::deque<char> new_input
	{
		'\x1'
	};
	serial_interface->consume_new_message(new_input);

	motor.run(); //Parse first byte
	clock->pass_time(16001);

	motor.run(); //Timer times out from not receiving second byte

	EXPECT_EQ(1, motor.modbus_client.diagnostic_counters[unexpected_interchar]);
}

TEST_F(ActuatorTests, AMessageMarkedImportantWillRetryEvenIfTheInitialMessageFailed)
{
	constexpr int unimportant_register_address = 180;
	motor.read_register(unimportant_register_address, MessagePriority::important);
	motor.run();
	clock->pass_time(50001);
	motor.run(); // Motor experiences time out

	EXPECT_EQ(1, motor.modbus_client.diagnostic_counters[return_server_no_response_count]);

	clock->pass_time(2001);

	motor.run(); // Motor should re-queue original read

	std::deque<char> returned_data {
		'\x1', '\x3', '\x2', '\x0', '\x5'
	};
	ModbusTesting::CalculateAndAppendCRC(returned_data);
	serial_interface->consume_new_message(returned_data);

	motor.run();

	EXPECT_EQ(5, motor.get_orca_reg_content(180));
}

TEST_F(ActuatorTests, SubsequentMessagesAfterAnImportantMessageAreNotAlsoMarkedImportant)
{
	constexpr int unimportant_register_address = 180;
	motor.read_register(unimportant_register_address, MessagePriority::important);
	motor.run();
	std::deque<char> returned_data{
		'\x1', '\x3', '\x2', '\x0', '\x5'
	};
	ModbusTesting::CalculateAndAppendCRC(returned_data);
	serial_interface->consume_new_message(returned_data);

	motor.run();

	clock->pass_time(2001);

	motor.read_register(unimportant_register_address, MessagePriority::not_important);

	motor.run(); 

	clock->pass_time(50001);
	
	motor.run();

	serial_interface->sendBuffer.clear();
	clock->pass_time(2001);

	motor.run();

	std::vector<char> out_buffer{};
	EXPECT_EQ(out_buffer, serial_interface->sendBuffer);
}

TEST_F(ActuatorTests, WhenStreamPauseIsCalledModbusHandshakeDoesntOccur)
{
	motor.enable();

	motor.set_stream_paused(true); // Disable the queuing of new stream messages

	motor.run(); // This sends the change mode command

	std::vector<char> out_buffer{};

	EXPECT_EQ(out_buffer, serial_interface->sendBuffer);
}

TEST_F(ActuatorTests, MultipleRegisterReadOfLengthZeroDoesNotGetQueued)
{
	std::vector<char> empty_out_buffer{};

	motor.read_registers(1, 0);
	motor.run();

	EXPECT_EQ(empty_out_buffer, serial_interface->sendBuffer);
}

TEST_F(ActuatorTests, MultipleRegisterReadOfLengthGreaterThan125DoesNotGetQueued)
{
	std::vector<char> empty_out_buffer{};

	motor.read_registers(1, 126);
	motor.run();

	EXPECT_EQ(empty_out_buffer, serial_interface->sendBuffer);
}

TEST_F(ActuatorTests, MultipleRegisterReadOfLength125GetsQueued)
{
	motor.read_registers(1, 125);
	motor.run();

	std::vector<char> empty_out_buffer{};

	EXPECT_NE(empty_out_buffer, serial_interface->sendBuffer);
}

TEST_F(ActuatorTests, MultipleRegisterWriteOfLengthZeroDoesNotGetQueued)
{
	std::vector<char> empty_out_buffer{};

	uint8_t unimportant_data[256];

	motor.write_registers(1, 0, unimportant_data);
	motor.run();

	EXPECT_EQ(empty_out_buffer, serial_interface->sendBuffer);
}

TEST_F(ActuatorTests, MultipleRegisterWriteOfLengthGreaterThan123DoesNotGetQueued)
{
	std::vector<char> empty_out_buffer{};

	uint8_t unimportant_data[256];

	motor.write_registers(1, 124, unimportant_data);
	motor.run();

	EXPECT_EQ(empty_out_buffer, serial_interface->sendBuffer);
}

TEST_F(ActuatorTests, MultipleRegisterWriteOfLength123GetsQueued)
{
	uint8_t unimportant_data[256];

	motor.write_registers(1, 123, unimportant_data);
	motor.run();

	std::vector<char> empty_out_buffer{};

	EXPECT_NE(empty_out_buffer, serial_interface->sendBuffer);
}