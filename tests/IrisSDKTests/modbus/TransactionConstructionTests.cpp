#include "pch.h"
#include "modbus_client/src/iris_client_application.h"
#include "modbus/helpers/TestModbusClient.h"
#include "modbus_client/actuator.h"

class TransactionConstructionTests : public testing::Test
{
protected:
	TransactionConstructionTests() :
		modbus_app(modbus_client, "Hello")
	{}

	TestModbusClient modbus_client;
	IrisClientApplication modbus_app;
};

TEST_F(TransactionConstructionTests, GetSingleRegisterCharacterizationTest)
{
	modbus_app.write_single_register_fn(1, 3, 25);
	modbus_client.run_out();
	std::vector<char> output{ 1, '\x6', '\0', 3, '\0', 25, '\xb8', '\0'};
	ASSERT_EQ(output, modbus_client.sendBuffer);
}

TEST_F(TransactionConstructionTests, IrisClientEnqueuePingMsgCharacterizationTest)
{
	modbus_app.read_holding_registers_fn(1, 780, 1);
	modbus_client.run_out();
	//								 Read register (0x3)	780 (256*0x3 + 0xc)			Length 1
	std::vector<char> output{ 1,	 '\x3',					'\x3', '\xc',		'\0',	'\x1', '\x44', '\x4d'};
	ASSERT_EQ(output, modbus_client.sendBuffer);
}