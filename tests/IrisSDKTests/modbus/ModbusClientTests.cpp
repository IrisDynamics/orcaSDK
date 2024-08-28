#include "pch.h"
#include "src/modbus_client.h"
#include "modbus/helpers/TestSerialInterface.h"
#include "modbus/helpers/TestLog.h"
#include "modbus/helpers/modbus_helpers.h"

class ModbusClientTests : public testing::Test
{
protected:
	ModbusClientTests() :
		log(std::make_shared<TestLog>()),
		modbus_client(serial_interface, -1)
	{
		modbus_client.begin_logging(log);
	}

	TestSerialInterface serial_interface;
	std::shared_ptr<TestLog> log;
	ModbusClient modbus_client;
};

TEST_F(ModbusClientTests, ModbusClientOutputsOutgoingBytesToLogFileIfLoggingEnabled)
{
	Transaction test_transaction;
	uint8_t data_bytes[2] = {
			'\x4',
			'\xc'
	};
	test_transaction.load_transmission_data(1, 3, data_bytes, 2, 6);

	modbus_client.enqueue_transaction(test_transaction);

	serial_interface.pass_time(10000);

	modbus_client.run_out();

	std::string logString = "10000\ttx\t01\t03\t04\t0c\tf3\t1d";
	ASSERT_EQ(logString, log->last_written_string);
}

TEST_F(ModbusClientTests, ModbusClientOutputsIncomingBytesToLogFileIfLoggingEnabled)
{
	Transaction test_transaction;
	uint8_t data_bytes[2] = {
			'\x4',
			'\xc'
	};
	test_transaction.load_transmission_data(1, 3, data_bytes, 2, 6);

	modbus_client.enqueue_transaction(test_transaction);

	serial_interface.pass_time(10000);

	modbus_client.run_out();

	std::deque<char> incoming_message{
		'\x1', '\x3', '\x4', '\xc', '\xf3', '\x1d'
	};
	serial_interface.consume_new_message(incoming_message);
	serial_interface.pass_time(500);
	modbus_client.run_in();

	std::string logString = "10500\trx\t01\t03\t04\t0c\tf3\t1d";
	ASSERT_EQ(logString, log->last_written_string);
}

TEST_F(ModbusClientTests, IfMessageTimedOutModbusClientAppendsTimedOutToLog)
{
	Transaction test_transaction;
	uint8_t data_bytes[2] = {
			'\x4',
			'\xc'
	};
	test_transaction.load_transmission_data(1, 3, data_bytes, 2, 6);

	modbus_client.enqueue_transaction(test_transaction);

	serial_interface.pass_time(10000);

	modbus_client.run_out();

	serial_interface.pass_time(DEFAULT_RESPONSE_uS + 1);
	modbus_client.run_in();

	std::string logString = "60001\trx\tTimed out. ";
	ASSERT_EQ(logString, log->last_written_string);
}

TEST_F(ModbusClientTests, AppendsUnexpectedIntercharTimeoutToLog)
{
	Transaction test_transaction;
	uint8_t data_bytes[2] = {
			'\x4',
			'\xc'
	};
	test_transaction.load_transmission_data(1, 3, data_bytes, 2, 6);

	modbus_client.enqueue_transaction(test_transaction);

	serial_interface.pass_time(10000);

	modbus_client.run_out();

	std::deque<char> incoming_message{
		'\x1'
	};
	serial_interface.consume_new_message(incoming_message);
	modbus_client.run_in();
	serial_interface.pass_time(DEFAULT_INTERCHAR_uS + 1);
	modbus_client.run_in();

	std::string logString = "26001\trx\t01\tUnexpected interchar. ";
	ASSERT_EQ(logString, log->last_written_string);
}

TEST_F(ModbusClientTests, AppendsWrongAddressWhenOutgoingAddressDoesntMatchIncomingAddress)
{
	Transaction test_transaction;
	uint8_t data_bytes[2] = {
			'\x4',
			'\xc'
	};
	test_transaction.load_transmission_data(1, 3, data_bytes, 2, 6);

	modbus_client.enqueue_transaction(test_transaction);

	serial_interface.pass_time(10000);

	modbus_client.run_out();
	
	std::deque<char> incoming_message{
		'\x2', '\x3', '\x4', '\xc'
	};
	ModbusTesting::CalculateAndAppendCRC(incoming_message);
	serial_interface.consume_new_message(incoming_message);
	modbus_client.run_in();

	std::string logString = "10000\trx\t02\t03\t04\t0c\tf3\t59\tWrong address. ";
	ASSERT_EQ(logString, log->last_written_string);
}

TEST_F(ModbusClientTests, AppendsWrongCRCWhenMessageCRCIsIncorrect)
{
	Transaction test_transaction;
	uint8_t data_bytes[2] = {
			'\x4',
			'\xc'
	};
	test_transaction.load_transmission_data(1, 3, data_bytes, 2, 6);

	modbus_client.enqueue_transaction(test_transaction);

	serial_interface.pass_time(10000);

	modbus_client.run_out();

	std::deque<char> incoming_message{
		'\x1', '\x3', '\x4', '\xc', '\x00', '\x00'
	};
	serial_interface.consume_new_message(incoming_message);
	modbus_client.run_in();

	std::string logString = "10000\trx\t01\t03\t04\t0c\t00\t00\tWrong CRC. ";
	ASSERT_EQ(logString, log->last_written_string);
}