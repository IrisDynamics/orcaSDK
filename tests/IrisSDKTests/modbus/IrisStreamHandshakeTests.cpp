#include "pch.h"
#include "src/iris_client_application.h"
#include "modbus/helpers/TestModbusClient.h"

class IrisStreamHandshakeTests : public testing::Test
{
protected:
	IrisStreamHandshakeTests() :
		modbus_app(modbus_client, "Hello")
	{}

	TestModbusClient modbus_client;
	IrisClientApplication modbus_app;

	void ReceiveMessageAndSendResponse(std::deque<char> message_to_receive)
	{
		modbus_client.sendBuffer.clear();
		modbus_client.consume_new_message(message_to_receive);
		modbus_client.run_in();
		modbus_app.consume_new_message();
		modbus_client.pass_time(2001); //Interframe delay is 2000us

		modbus_app.modbus_handshake();
		modbus_client.run_out();
	}
};

TEST_F(IrisStreamHandshakeTests, FirstHandshakeMessageIsPing)
{
	modbus_app.modbus_handshake();
	modbus_client.run_out();
	std::vector<char> output{ '\x1', '\b', '\0', '\0', '\x80', '\x1a'};
	ASSERT_EQ(output, modbus_client.sendBuffer);
}

TEST_F(IrisStreamHandshakeTests, SecondHandshakeOnlySendsDataIfReceivedResponse)
{
	modbus_app.modbus_handshake();
	modbus_client.run_out();
	modbus_client.sendBuffer.clear();
	modbus_app.modbus_handshake();
	modbus_client.run_out();
	ASSERT_EQ(std::vector<char>{}, modbus_client.sendBuffer);
}

TEST_F(IrisStreamHandshakeTests, SecondHandshakePingsAgainIfReceivedEcho)
{
	modbus_app.modbus_handshake();
	modbus_client.run_out();
	modbus_client.sendBuffer.clear();

	ReceiveMessageAndSendResponse({ '\x1', '\b', '\0', '\0', '\x80', '\x1a' });

	std::vector<char> output = { '\x1', '\b', '\0', '\0', '\x80', '\x1a' };
	ASSERT_EQ(output, modbus_client.sendBuffer);
}

TEST_F(IrisStreamHandshakeTests, AfterThreeDiscoveryPingsSendsManageHighSpeedStreamRequest)
{
	modbus_app.modbus_handshake();
	modbus_client.run_out(); //First echo sent

	//1
	ReceiveMessageAndSendResponse({ '\x1', '\b', '\0', '\0', '\x80', '\x1a' });

	//2
	ReceiveMessageAndSendResponse({ '\x1', '\b', '\0', '\0', '\x80', '\x1a' });

	//3
	ReceiveMessageAndSendResponse({ '\x1', '\b', '\0', '\0', '\x80', '\x1a' });

	//Send manage high-speed stream 
	modbus_app.modbus_handshake();
	modbus_client.run_out(); //Sends Manage High-speed Stream message 

	std::vector<char> output = {
		'\x01', //Orca address
		65, //Manage High-speed Stream message ID
		'\xff', '\0', // Enable and apply parameters
		'\0', '\x9', '\x89', '\x68', // Target baud rate (650000bps)
		'\0', '\x50', // Target Response delay (80ms)
		'\x25', '\x28' // CRC
	};
	ASSERT_EQ(output, modbus_client.sendBuffer);
}

TEST_F(IrisStreamHandshakeTests, AfterSendingManageHighSpeedStreamMessageSetsConnectedAndUpdatesBaud)
{
	modbus_app.modbus_handshake();
	modbus_client.run_out(); //First echo sent

	// Discovery 1
	ReceiveMessageAndSendResponse({ '\x1', '\b', '\0', '\0', '\x80', '\x1a' });

	//2
	ReceiveMessageAndSendResponse({ '\x1', '\b', '\0', '\0', '\x80', '\x1a' });

	//3
	ReceiveMessageAndSendResponse({ '\x1', '\b', '\0', '\0', '\x80', '\x1a' });

	//Send manage high-speed stream 
	modbus_app.modbus_handshake();
	modbus_client.run_out(); //Sends Manage High-speed Stream message 

	modbus_client.consume_new_message(
		{
			'\x01', //Orca address
			65, //Manage High-speed Stream message ID
			'\xff', '\0', // Echo of enable and apply parameters
			'\0', '\0', '\0', '\xff', // Target baud rate (255bps)
			'\0', '\xa0', // Target Response delay (128ms)
			'\x62', '\xdd' // CRC calculated for custom response
		});
	modbus_client.run_in();
	modbus_app.consume_new_message();

	//Handle manage high-speed stream response and move to connected state
	modbus_app.modbus_handshake();

	ASSERT_TRUE(modbus_app.is_connected());
	ASSERT_EQ(255, modbus_client.adjusted_baud_rate);
}