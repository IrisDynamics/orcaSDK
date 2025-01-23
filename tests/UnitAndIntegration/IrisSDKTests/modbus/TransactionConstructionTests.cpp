#include "pch.h"
#include "src/standard_modbus_functions.h"
#include "modbus/helpers/TestSerialInterface.h"
#include "modbus/helpers/TestClock.h"
#include "actuator.h"

using namespace orcaSDK;

class TransactionConstructionTests : public testing::Test
{
protected:
	TransactionConstructionTests() :
		modbus_client(serial_interface, clock, -1)
	{}

	TestSerialInterface serial_interface;
	TestClock clock;
	ModbusClient modbus_client;
};

TEST_F(TransactionConstructionTests, GetSingleRegisterCharacterizationTest)
{
	modbus_client.enqueue_transaction(DefaultModbusFunctions::write_single_register_fn(1, 3, 25, MessagePriority::not_important));
	modbus_client.run_out();
	std::vector<char> output{ 1, '\x6', '\0', 3, '\0', 25, '\xb8', '\0'};
	ASSERT_EQ(output, serial_interface.sendBuffer);
}

TEST_F(TransactionConstructionTests, IrisClientEnqueuePingMsgCharacterizationTest)
{
	modbus_client.enqueue_transaction(DefaultModbusFunctions::read_holding_registers_fn(1, 780, 1, MessagePriority::not_important));
	modbus_client.run_out();
	//								 Read register (0x3)	780 (256*0x3 + 0xc)			Length 1
	std::vector<char> output{ 1,	 '\x3',					'\x3', '\xc',		'\0',	'\x1', '\x44', '\x4d'};
	ASSERT_EQ(output, serial_interface.sendBuffer);
}