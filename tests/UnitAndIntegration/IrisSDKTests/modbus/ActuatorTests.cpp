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
		clock->set_auto_time_pass(2001); //Automatically pass interframe delay
	}

	std::shared_ptr<TestSerialInterface> serial_interface;
	std::shared_ptr<TestClock> clock;
	std::shared_ptr<TestLog> log;
	Actuator motor;
};

TEST_F(ActuatorTests, ReadingFromMemoryMapAndThenReceivingUpdatesLocalMemoryMap)
{
	std::deque<char> next_input_message = std::deque<char>{
		//						 low  reg(2)  high reg(65536)
			'\x1', '\x3', '\x4', '\0', '\x2', '\0', '\x1'
	};
	ModbusTesting::CalculateAndAppendCRC(next_input_message);
	serial_interface->consume_new_message(next_input_message);

	int32_t output_position = motor.read_wide_register_blocking(SHAFT_POS_UM).value;

	std::vector<char> expected_sent_buffer{
		'\x1', '\x3', '\x1', '\x56', '\0', '\x2', '\x25', '\xe7'
	};
	EXPECT_EQ(expected_sent_buffer, serial_interface->sendBuffer);

	EXPECT_EQ(65538, output_position);
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
	serial_interface->sendBuffer.clear();
	serial_interface->consume_new_message(ping_echo);
	motor.run();
	serial_interface->sendBuffer.clear();
	serial_interface->consume_new_message(ping_echo);
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

	EXPECT_TRUE(motor.is_connected());
	EXPECT_EQ(625000, serial_interface->adjusted_baud_rate);
}

TEST_F(ActuatorTests, MotorIncrementsCRCDiagnosticCounterOnBadCRCResponse)
{
	std::deque<char> receive_buffer{
		'\x1', '\x3', '\x1', '\x0', '\xff', '\x0', '\x0'
	};
	serial_interface->consume_new_message(receive_buffer);

	auto [_, error] = motor.read_register_blocking(POWER, MessagePriority::not_important);

	EXPECT_EQ(1 << Transaction::CRC_ERROR, error.failure_codes());
}

TEST_F(ActuatorTests, MotorIncrementsTimeoutAfterEnoughTimePassesBetweenSeeingFullMessageResponseTimeout)
{
	auto [_, error] = motor.read_register_blocking(POWER, MessagePriority::not_important);

	EXPECT_EQ(1 << Transaction::RESPONSE_TIMEOUT_ERROR, error.failure_codes());
}

TEST_F(ActuatorTests, MotorIncrementsIntercharTimeoutAfterEnoughTimePassesBetweenSeeingNewBytes)
{
	std::deque<char> new_input
	{
		'\x1'
	};
	serial_interface->consume_new_message(new_input);

	auto [_, error] = motor.read_register_blocking(POWER, MessagePriority::not_important);

	EXPECT_EQ(1 << Transaction::INTERCHAR_TIMEOUT_ERROR, error.failure_codes());
}

TEST_F(ActuatorTests, AMessageMarkedImportantWillRetryEvenIfTheInitialMessageFailed)
{
	motor.read_register_blocking(POWER, MessagePriority::important);

	EXPECT_EQ(6, motor.modbus_client.diagnostic_counters[return_server_no_response_count]);
}

TEST_F(ActuatorTests, SubsequentMessagesAfterAnImportantMessageAreNotAlsoMarkedImportant)
{
	motor.read_register_blocking(POWER, MessagePriority::important);

	EXPECT_EQ(6, motor.modbus_client.diagnostic_counters[return_server_no_response_count]);

	motor.read_register_blocking(POWER, MessagePriority::not_important);

	EXPECT_EQ(7, motor.modbus_client.diagnostic_counters[return_server_no_response_count]);
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

	motor.read_multiple_registers_blocking(1, 0);

	EXPECT_EQ(empty_out_buffer, serial_interface->sendBuffer);
}

TEST_F(ActuatorTests, MultipleRegisterWriteOfLengthZeroDoesNotGetQueued)
{
	std::vector<char> empty_out_buffer{};

	uint16_t unimportant_data[256];

	motor.write_multiple_registers_blocking(1, 0, unimportant_data);

	EXPECT_EQ(empty_out_buffer, serial_interface->sendBuffer);
}

//TEST_F(ActuatorTests, MultipleRegisterReadOfLengthGreaterThan125DoesNotGetQueued)
//{
//	std::vector<char> empty_out_buffer{};
//
//	motor.read_registers(1, 126);
//	motor.run();
//
//	EXPECT_EQ(empty_out_buffer, serial_interface->sendBuffer);
//}

//TEST_F(ActuatorTests, MultipleRegisterReadOfLength125GetsQueued)
//{
//	motor.read_registers(1, 125);
//	motor.run();
//
//	std::vector<char> empty_out_buffer{};
//
//	EXPECT_NE(empty_out_buffer, serial_interface->sendBuffer);
//}

//TEST_F(ActuatorTests, MultipleRegisterWriteOfLengthGreaterThan123DoesNotGetQueued)
//{
//	std::vector<char> empty_out_buffer{};
//
//	uint8_t unimportant_data[256];
//
//	motor.write_registers(1, 124, unimportant_data);
//	motor.run();
//
//	EXPECT_EQ(empty_out_buffer, serial_interface->sendBuffer);
//}
//
//TEST_F(ActuatorTests, MultipleRegisterWriteOfLength123GetsQueued)
//{
//	uint8_t unimportant_data[256];
//
//	motor.write_registers(1, 123, unimportant_data);
//	motor.run();
//
//	std::vector<char> empty_out_buffer{};
//
//	EXPECT_NE(empty_out_buffer, serial_interface->sendBuffer);
//}