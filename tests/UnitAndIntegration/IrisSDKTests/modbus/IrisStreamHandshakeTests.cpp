#include "pch.h"
#include "actuator.h"
#include "modbus/helpers/TestSerialInterface.h"
#include "modbus/helpers/TestClock.h"

class IrisStreamHandshakeTests : public testing::Test
{
protected:
	IrisStreamHandshakeTests() :
		//modbus_client(serial_interface, -1),
		serial_interface(std::make_shared<TestSerialInterface>()),
		clock(std::make_shared<TestClock>()),
		modbus_app(
			serial_interface,
			clock,
			0,
			"unimportant")
	{}

	void SetUp()
	{
		modbus_app.enable();
	}

	std::shared_ptr<TestSerialInterface> serial_interface;
	std::shared_ptr<TestClock> clock;
	Actuator modbus_app;

	void ReceiveMessageAndSendResponse(std::deque<char> message_to_receive)
	{
		serial_interface->sendBuffer.clear();
		serial_interface->consume_new_message(message_to_receive);
		modbus_app.run();
		clock->pass_time(2001); //Interframe delay is 2000us

		modbus_app.run();
	}
};

TEST_F(IrisStreamHandshakeTests, FirstHandshakeMessageIsPing)
{
	modbus_app.run();
	std::vector<char> output{ '\x1', '\b', '\0', '\0', '\x80', '\x1a'};
	ASSERT_EQ(output, serial_interface->sendBuffer);
}

//Deceptive test. Multiple factors lead to the second one not pinging, even if it queued another ping
// Not certain the value of this
TEST_F(IrisStreamHandshakeTests, SecondHandshakeDoesntSendPing) 
{
	modbus_app.run();
	serial_interface->sendBuffer.clear();
	modbus_app.run();
	ASSERT_EQ(std::vector<char>{}, serial_interface->sendBuffer);
}

TEST_F(IrisStreamHandshakeTests, SecondHandshakePingsAgainIfReceivedEcho)
{
	modbus_app.run();
	serial_interface->sendBuffer.clear();

	ReceiveMessageAndSendResponse({ '\x1', '\b', '\0', '\0', '\x80', '\x1a' });

	std::vector<char> output = { '\x1', '\b', '\0', '\0', '\x80', '\x1a' };
	ASSERT_EQ(output, serial_interface->sendBuffer);
}