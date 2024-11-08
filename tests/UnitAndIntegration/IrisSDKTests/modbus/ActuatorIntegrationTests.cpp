#include "pch.h"
#include "helpers/TestSerialInterface.h"
#include "helpers/TestClock.h"
#include "helpers/modbus_helpers.h"
#include "actuator.h"

//using ::testing::Return;
//using ::testing::NiceMock;
//using ::testing::InSequence;

class ActuatorIntegrationTests : public testing::Test
{
protected:
	ActuatorIntegrationTests() :
		serial_interface(std::make_shared<TestSerialInterface>()),
		clock(std::make_shared<TestClock>()),
		motor(serial_interface, clock, -1, "unimportant")
	{}

	void SetUp() {}

	std::shared_ptr<TestSerialInterface> serial_interface;
	std::shared_ptr<TestClock> clock;
	Actuator motor;
};

TEST_F(ActuatorIntegrationTests, PerformingReadReturnsValueReadFromSerialPort)
{
	std::deque<char> next_input_message = std::deque<char>{
		//						 low  reg(2)  high reg(65536)
			'\x1', '\x3', '\x4', '\0', '\x2', '\0', '\x1'
	};
	ModbusTesting::CalculateAndAppendCRC(next_input_message);
	serial_interface->consume_new_message(next_input_message);

	EXPECT_EQ(65538, motor.read_wide_register_blocking(SHAFT_POS_UM));
}

TEST_F(ActuatorIntegrationTests, ReadWideRegisterBlockingSendsCorrectMessage)
{
	clock->set_auto_time_pass(10000); //Force timeout, only care about send

	motor.read_wide_register_blocking(SHAFT_POS_UM, MessagePriority::not_important);

	std::vector<char> expected_send{
		'\x1', '\x3', '\x1', '\x56', '\x0', '\x2', '\x25', '\xe7',
	};

	EXPECT_EQ(expected_send, serial_interface->sendBuffer);
}

TEST_F(ActuatorIntegrationTests, PerformingSingleWideReadReturnsSingleWideReturn)
{
	std::deque<char> next_input_message = std::deque<char>{
		//						 low  reg(2)
			'\x1', '\x3', '\x4', '\0', '\x2'
	};
	ModbusTesting::CalculateAndAppendCRC(next_input_message);
	serial_interface->consume_new_message(next_input_message);

	EXPECT_EQ(2, motor.read_register_blocking(STATOR_TEMP));
}

TEST_F(ActuatorIntegrationTests, PerformingMultipleReadReturnsVectorOfResults)
{
	std::deque<char> next_input_message = std::deque<char>{
		//						 low  reg(2)
			'\x1', '\x3', '\x4', '\0', '\x2', '\x1', '\x0', '\x0', '\xff'
	};
	ModbusTesting::CalculateAndAppendCRC(next_input_message);
	serial_interface->consume_new_message(next_input_message);

	std::vector<uint16_t> expected_output { 2, 256, 255 };

	EXPECT_EQ(expected_output, motor.read_multiple_registers_blocking(STATOR_TEMP, 3));
}

TEST_F(ActuatorIntegrationTests, PerformingWriteSendsSingleByteWriteMessage)
{
	clock->set_auto_time_pass(10000); //Force timeout, only care about send

	motor.write_register_blocking(SHAFT_POS_UM, 45, MessagePriority::not_important);

	std::vector<char> expected_send{
		'\x1', '\x6', '\x1', '\x56', '\x0', '\x2d', '\xa8', '\x3b',
	};

	EXPECT_EQ(expected_send, serial_interface->sendBuffer);
}

TEST_F(ActuatorIntegrationTests, PerformingMultipleWriteSendsMultipleWriteMessage)
{
	clock->set_auto_time_pass(10000); //Force timeout, only care about send

	std::array<uint16_t, 2> out_values =
	{
		256,
		512
	};
	motor.write_multiple_registers_blocking(SHAFT_POS_UM, 2, out_values.data(), MessagePriority::not_important);

	std::vector<char> expected_send{
		'\x1', '\x10', '\x1', '\x56', '\x0', '\x2', '\x4', '\x1', '\x0', '\x2', '\x0', '\x7b', '\xb5',
	};

	EXPECT_EQ(expected_send, serial_interface->sendBuffer);
}

TEST_F(ActuatorIntegrationTests, ReadWriteRegistersBlockingBothWritesCorrectlyAndReadsCorrectly)
{
	//Arrange
	std::array<uint16_t, 2> input_data { 16, 512 };

	std::deque<char> serial_read_data = std::deque<char>{
		'\x1', '\x3', '\x4', '\0', '\x2', '\x1', '\x0', '\x0', '\xff'
	};
	ModbusTesting::CalculateAndAppendCRC(serial_read_data);
	serial_interface->consume_new_message(serial_read_data);

	//Act
	std::vector<uint16_t> actual_output = motor.read_write_multiple_registers_blocking(
		128, 3, 256, 2, input_data.data());

	//Assert
	std::vector<char> expected_serial_write_data = std::vector<char>{
			'\x1', '\x17',
			'\x0', '\x80',	//Read start address 
			'\x0', '\x3',	//Read num registers
			'\x1', '\x0',	//Write start address
			'\x0', '\x2',	//Write num registers
			'\x4',			//Write byte count
			'\x0', '\x10', '\x2', '\x0',	//Write data
			'\x99', '\xce'	//CRC
	};
	EXPECT_EQ(expected_serial_write_data, serial_interface->sendBuffer);

	std::vector<uint16_t> expected_output{ 2, 256, 255 };
	EXPECT_EQ(expected_output, actual_output);
}

//TEST_F()
//{
//	motor.read_registers(SHAFT_POS_UM, 2);
//	motor.run();
//
//	std::vector<char> expected_sent_buffer{
//		'\x1', '\x3', '\x1', '\x56', '\0', '\x2', '\x25', '\xe7'
//	};
//
//	EXPECT_EQ(expected_sent_buffer, serial_interface->sendBuffer);
//
//	std::deque<char> next_input_message = std::deque<char>{
//		//						 low  reg(2)  high reg(65536)
//			'\x1', '\x3', '\x4', '\0', '\x2', '\0', '\x1'
//	};
//	ModbusTesting::CalculateAndAppendCRC(next_input_message);
//	serial_interface->consume_new_message(next_input_message);
//
//	motor.run();
//
//	EXPECT_EQ(65538, motor.get_position_um());
//}