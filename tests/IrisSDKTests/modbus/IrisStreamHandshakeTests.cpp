#include "pch.h"
#include "actuator.h"
#include "modbus/helpers/TestSerialInterface.h"

class IrisStreamHandshakeTests : public testing::Test
{
protected:
	IrisStreamHandshakeTests() :
		//modbus_client(serial_interface, -1),
		serial_interface(std::make_shared<TestSerialInterface>()),
		modbus_app(
			serial_interface,
			log, 
			0,
			"Hello")
	{}

	std::shared_ptr<TestSerialInterface> serial_interface;
	std::shared_ptr<Log> log;
	//ModbusClient modbus_client;
	Actuator modbus_app;

	void ReceiveMessageAndSendResponse(std::deque<char> message_to_receive)
	{
		serial_interface->sendBuffer.clear();
		serial_interface->consume_new_message(message_to_receive);
		modbus_app.modbus_client.run_in();
		modbus_app.consume_new_message();
		serial_interface->pass_time(2001); //Interframe delay is 2000us

		modbus_app.modbus_handshake();
		modbus_app.modbus_client.run_out();
	}
};

TEST_F(IrisStreamHandshakeTests, FirstHandshakeMessageIsPing)
{
	modbus_app.modbus_handshake();
	modbus_app.modbus_client.run_out();
	std::vector<char> output{ '\x1', '\b', '\0', '\0', '\x80', '\x1a'};
	ASSERT_EQ(output, serial_interface->sendBuffer);
}

TEST_F(IrisStreamHandshakeTests, SecondHandshakeOnlySendsDataIfReceivedResponse)
{
	modbus_app.modbus_handshake();
	modbus_app.modbus_client.run_out();
	serial_interface->sendBuffer.clear();
	modbus_app.modbus_handshake();
	modbus_app.modbus_client.run_out();
	ASSERT_EQ(std::vector<char>{}, serial_interface->sendBuffer);
}

TEST_F(IrisStreamHandshakeTests, SecondHandshakePingsAgainIfReceivedEcho)
{
	modbus_app.modbus_handshake();
	modbus_app.modbus_client.run_out();
	serial_interface->sendBuffer.clear();

	ReceiveMessageAndSendResponse({ '\x1', '\b', '\0', '\0', '\x80', '\x1a' });

	std::vector<char> output = { '\x1', '\b', '\0', '\0', '\x80', '\x1a' };
	ASSERT_EQ(output, serial_interface->sendBuffer);
}