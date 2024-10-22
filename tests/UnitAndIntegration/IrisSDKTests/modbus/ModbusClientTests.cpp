#include "pch.h"
#include "src/modbus_client.h"
#include "modbus/helpers/TestSerialInterface.h"
#include "modbus/helpers/TestLog.h"
#include "modbus/helpers/modbus_helpers.h"
#include "modbus/helpers/TestClock.h"

class ModbusClientTests : public testing::Test
{
protected:
	ModbusClientTests() :
		log(std::make_shared<TestLog>()),
		modbus_client(serial_interface, clock, -1)
	{
		modbus_client.begin_logging(log);
	}

	TestSerialInterface serial_interface;
	TestClock clock;
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

	clock.pass_time(10000);

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

	clock.pass_time(10000);

	modbus_client.run_out();

	std::deque<char> incoming_message{
		'\x1', '\x3', '\x4', '\xc', '\xf3', '\x1d'
	};
	serial_interface.consume_new_message(incoming_message);
	clock.pass_time(500);
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

	clock.pass_time(10000);

	modbus_client.run_out();

	clock.pass_time(DEFAULT_RESPONSE_uS + 1);
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

	clock.pass_time(10000);

	modbus_client.run_out();

	std::deque<char> incoming_message{
		'\x1'
	};
	serial_interface.consume_new_message(incoming_message);
	modbus_client.run_in();
	clock.pass_time(DEFAULT_INTERCHAR_uS + 1);
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

	clock.pass_time(10000);

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

	clock.pass_time(10000);

	modbus_client.run_out();

	std::deque<char> incoming_message{
		'\x1', '\x3', '\x4', '\xc', '\x00', '\x00'
	};
	serial_interface.consume_new_message(incoming_message);
	modbus_client.run_in();

	std::string logString = "10000\trx\t01\t03\t04\t0c\t00\t00\tWrong CRC. ";
	ASSERT_EQ(logString, log->last_written_string);
}

TEST_F(ModbusClientTests, IfAnImportantMessageFailsForAnyReasonARetryTransactionIsGenerated)
{
	Transaction test_transaction;
	test_transaction.mark_important();
	uint8_t data_bytes[2] = {
			'\x4',
			'\xc'
	};
	test_transaction.load_transmission_data(1, 3, data_bytes, 2, 6);

	modbus_client.enqueue_transaction(test_transaction);
	modbus_client.run_out();

	std::deque<char> incoming_message{
		'\x1', '\x3', '\x4', '\xc', '\x00', '\x00'
	};
	serial_interface.consume_new_message(incoming_message);
	modbus_client.run_in();

	ASSERT_EQ(1, modbus_client.diagnostic_counters.Get(diagnostic_counter_t::crc_error_count)); // Ensure the transaction failed

	modbus_client.dequeue_transaction();

	serial_interface.sendBuffer.clear();
	clock.pass_time(2001);
	modbus_client.run_out();

	std::vector<char> expected_output{ '\x1', '\x3', '\x4', '\xc', '\xf3', '\x1d' };
	ASSERT_EQ(expected_output, serial_interface.sendBuffer);
}

TEST_F(ModbusClientTests, ImportantMessagesAreGivenUpOnAfterFiveFailedRetries)
{
	Transaction test_transaction;
	test_transaction.mark_important();
	uint8_t data_bytes[2] = {
			'\x4',
			'\xc'
	};
	test_transaction.load_transmission_data(1, 3, data_bytes, 2, 6);

	modbus_client.enqueue_transaction(test_transaction);
	modbus_client.run_out();

	constexpr int num_allowed_retries = 5;
	for (int i = 0; i < num_allowed_retries + 1; i++)
	{
		std::deque<char> incoming_message{
			'\x1', '\x3', '\x4', '\xc', '\x00', '\x00'
		};
		serial_interface.consume_new_message(incoming_message);
		modbus_client.run_in();

		modbus_client.dequeue_transaction();

		serial_interface.sendBuffer.clear();
		clock.pass_time(2001);
		modbus_client.run_out();
	}

	std::vector<char> expected_output{};
	ASSERT_EQ(expected_output, serial_interface.sendBuffer);
}

TEST_F(ModbusClientTests, ImportantMessageRetriesAreInsertedAtTheFrontOfTheQueue)
{
	//Insert transaction one
	Transaction test_transaction_one;
	test_transaction_one.mark_important();
	uint8_t data_bytes_0[2] = {
			'\x4',
			'\xc'
	};
	test_transaction_one.load_transmission_data(1, 3, data_bytes_0, 2, 6);

	modbus_client.enqueue_transaction(test_transaction_one);
	modbus_client.run_out();

	//Insert transaction two
	Transaction test_transaction_two;
	uint8_t data_bytes_1[2] = {
			'\x1',
			'\x1'
	};
	test_transaction_two.load_transmission_data(1, 3, data_bytes_1, 2, 6);

	modbus_client.enqueue_transaction(test_transaction_two);
	modbus_client.run_out();


	std::deque<char> incoming_message{
		'\x1', '\x3', '\x4', '\xc', '\x00', '\x00'
	};
	serial_interface.consume_new_message(incoming_message);
	modbus_client.run_in();

	ASSERT_EQ(1, modbus_client.diagnostic_counters.Get(diagnostic_counter_t::crc_error_count)); // Ensure the transaction failed

	modbus_client.dequeue_transaction();

	serial_interface.sendBuffer.clear();
	clock.pass_time(2001);
	modbus_client.run_out();

	// Expect retry of transaction one
	std::vector<char> expected_output{ '\x1', '\x3', '\x4', '\xc', '\xf3', '\x1d' };
	ASSERT_EQ(expected_output, serial_interface.sendBuffer);
}