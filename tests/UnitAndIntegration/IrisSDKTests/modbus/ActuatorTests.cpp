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
		motor(serial_interface, clock, "unimportant")
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

TEST_F(ActuatorTests, MotorIncrementsCRCDiagnosticCounterOnBadCRCResponse)
{
	std::deque<char> receive_buffer{
		'\x1', '\x3', '\x1', '\x0', '\xff', '\x0', '\x0'
	};
	serial_interface->consume_new_message(receive_buffer);

	OrcaResult<uint16_t> result = motor.read_register_blocking(POWER, MessagePriority::not_important);

	EXPECT_TRUE(result.error);
}

TEST_F(ActuatorTests, MotorIncrementsTimeoutAfterEnoughTimePassesBetweenSeeingFullMessageResponseTimeout)
{
	OrcaResult<uint16_t> result = motor.read_register_blocking(POWER, MessagePriority::not_important);

	EXPECT_TRUE(result.error);
}

TEST_F(ActuatorTests, MotorIncrementsIntercharTimeoutAfterEnoughTimePassesBetweenSeeingNewBytes)
{
	std::deque<char> new_input
	{
		'\x1'
	};
	serial_interface->consume_new_message(new_input);

	OrcaResult<uint16_t> result = motor.read_register_blocking(POWER, MessagePriority::not_important);

	EXPECT_TRUE(result.error);
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

TEST_F(ActuatorTests, TimeSinceLastMessageBeginsVeryLarge)
{
	EXPECT_GT(motor.time_since_last_response_microseconds(), 1000000000);
}

TEST_F(ActuatorTests, AfterReceivingASuccessfulMessageTimeSinceLastMessageBecomesZero)
{
	clock->set_auto_time_pass(0);

	std::deque<char> next_input_message = std::deque<char>{
		//						 low  reg(2)
			'\x1', '\x3', '\x4', '\0', '\x2'
	};
	ModbusTesting::CalculateAndAppendCRC(next_input_message);
	serial_interface->consume_new_message(next_input_message);

	motor.read_register_blocking(STATOR_TEMP);

	EXPECT_EQ(motor.time_since_last_response_microseconds(), 0);
}

TEST_F(ActuatorTests, AfterReceivingASuccessfulMessageAndWaitingTimeSinceLastResponseMatchesTimeWaited)
{
	clock->set_auto_time_pass(0);

	std::deque<char> next_input_message = std::deque<char>{
		//						 low  reg(2)
			'\x1', '\x3', '\x4', '\0', '\x2'
	};
	ModbusTesting::CalculateAndAppendCRC(next_input_message);
	serial_interface->consume_new_message(next_input_message);

	motor.read_register_blocking(STATOR_TEMP);

	clock->pass_time(5000);

	EXPECT_EQ(motor.time_since_last_response_microseconds(), 5000);
}

TEST_F(ActuatorTests, InvalidMessagesDoNotUpdateTimeSinceLastResponse)
{
	OrcaResult<uint16_t> result = motor.read_register_blocking(POWER, MessagePriority::not_important);

	EXPECT_GT(motor.time_since_last_response_microseconds(), 1000000000);
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