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
		motor(modbus_client, -1, "Hello")
	{}

	std::shared_ptr<TestModbusClient> modbus_client = std::make_shared<TestModbusClient>();
	Actuator motor;
};

TEST_F(ActuatorTests, ReadingFromMemoryMapAndThenReceivingUpdatesLocalMemoryMap)
{
	motor.read_registers(SHAFT_POS_UM, 2);
	motor.run_out();
	
	std::vector<char> expected_sent_buffer{
		'\x1', '\x3', '\x1', '\x56', '\0', '\x2', '\x25', '\xe7'
	};

	EXPECT_EQ(expected_sent_buffer, modbus_client->sendBuffer);

	std::deque<char> next_input_message = std::deque<char>{
		//						 low  reg(2)  high reg(65536)
			'\x1', '\x3', '\x4', '\0', '\x2', '\0', '\x1'
	};
	ModbusTesting::CalculateAndAppendCRC(next_input_message);
	modbus_client->consume_new_message(next_input_message);

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
	EXPECT_EQ(ping_message, modbus_client->sendBuffer);

	//-----Discovery-----
	// Involves sending three ping messages and hearing three echo responses
	// After performing this back and forth, enqueues messages to synchronize
	// the local memory map, and enters beginning of synchronization
	std::deque<char> ping_echo;
	for (char c : ping_message)
	{
		ping_echo.push_back(c);
	}
	modbus_client->sendBuffer.clear();
	modbus_client->consume_new_message(ping_echo);
	modbus_client->pass_time(2001);
	motor.run_in();
	motor.run_out();
	modbus_client->sendBuffer.clear();
	modbus_client->consume_new_message(ping_echo);
	modbus_client->pass_time(2001);
	motor.run_in();
	motor.run_out();
	modbus_client->sendBuffer.clear();
	modbus_client->consume_new_message(ping_echo);
	modbus_client->pass_time(2001);
	motor.run_in();
	motor.run_out();

	//-----Synchronization-----
	// Sends messages asking for memory reads for different memory regions
	// Completes when each read returns with the correct shape and parameters
	// and Actuator::run_in() consumes all 3 messages
	std::vector<char> first_sync_message{ '\x1', '\x03', '\x01', '\x90', '\0', '\x13', '\x5', '\xd6' };
	EXPECT_EQ(first_sync_message, modbus_client->sendBuffer);

	std::deque<char> first_sync_response{
		'\x1', '\x3', '\x13'
	};
	for (int i = 0; i < 0x13 * 2; i++) // Push 19 (0x13) register values into the receive buffer
	{
		first_sync_response.push_back('\0');
	}
	ModbusTesting::CalculateAndAppendCRC(first_sync_response);
	modbus_client->consume_new_message(first_sync_response);

	modbus_client->sendBuffer.clear();
	modbus_client->pass_time(2001);
	motor.run_out();

	std::vector<char> second_sync_message{ '\x1', '\x03', '\x01', '\xb0', '\0', '\x5', '\x85', '\xd2' };
	EXPECT_EQ(second_sync_message, modbus_client->sendBuffer);

	std::deque<char> second_sync_response{
		'\x1', '\x3', '\x5'
	};
	for (int i = 0; i < 0x5 * 2; i++) // Push 5 register values into the receive buffer
	{
		second_sync_response.push_back('\0');
	}
	ModbusTesting::CalculateAndAppendCRC(second_sync_response);
	modbus_client->consume_new_message(second_sync_response);

	modbus_client->sendBuffer.clear();
	modbus_client->pass_time(2001);
	motor.run_out();

	std::vector<char> third_sync_message{ '\x1', '\x03', '\0', '\x80', '\0', '\x19', '\x85', '\xe8' };
	EXPECT_EQ(third_sync_message, modbus_client->sendBuffer);

	std::deque<char> third_sync_response{
		'\x1', '\x3', '\x19'
	};
	for (int i = 0; i < 0x19 * 2; i++) // Push 24 (0x18) register values into the receive buffer
	{
		third_sync_response.push_back('\0');
	}
	ModbusTesting::CalculateAndAppendCRC(third_sync_response);
	modbus_client->consume_new_message(third_sync_response);
	
	//Consume the 3 ready response messages.
	motor.run_in();
	motor.run_in();
	motor.run_in();

	modbus_client->sendBuffer.clear();
	modbus_client->pass_time(2001);
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
	modbus_client->consume_new_message(stream_negotiation_response);
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

	EXPECT_EQ(expected_sent_buffer, modbus_client->sendBuffer);

	std::deque<char> expected_response = std::deque<char>{
			'\x01', '\x17', '\x04', '\x0d', '\x9d', '\x0e', '\x3e'
	};
	ModbusTesting::CalculateAndAppendCRC(expected_response);
	modbus_client->consume_new_message(expected_response);

	motor.run_in();

	EXPECT_EQ(motor.get_orca_reg_content(97), 0xd9d);
	EXPECT_EQ(motor.get_orca_reg_content(98), 0xe3e);
}

TEST_F(ActuatorTests, ModbusMotorCommandCorrectlyHandlesBadResponse)
{

}

//TEST_F(ActuatorTests, ModbusFlushUpdatesRegistersImmediately)
//{
//	motor.read_registers(SHAFT_POS_UM, 2);
//
//	std::deque<char> next_input_message = std::deque<char>{
//		//						 low  reg(2)  high reg(65536)
//			'\x1', '\x3', '\x4', '\0', '\x2', '\0', '\x1'
//	};
//	ModbusTesting::CalculateAndAppendCRC(next_input_message);
//	modbus_client->consume_new_message(next_input_message);
//
//	motor.flush();
//	EXPECT_EQ(65538, motor.get_position_um());
//}
