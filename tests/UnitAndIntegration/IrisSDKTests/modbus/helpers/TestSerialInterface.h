#pragma once

#include "pch.h"
#include "src/modbus_client.h"
#include <vector>
#include <deque>

class TestSerialInterface : public orcaSDK::SerialInterface
{
public:
	TestSerialInterface()
	{}

	void prepare_new_message(std::deque<char> new_message)
	{
		receive_buffer = new_message;
	}

	void consume_new_message(std::deque<char> new_message)
	{
		receive_buffer = new_message;
	}

	std::vector<char> sendBuffer;
	int adjusted_baud_rate = 0;


	//Handling TX
	void tx_enable(size_t) override {
		////while there are bytes left to send in the transaction, continue adding them to sendBuf
		//while (messages.get_active_transaction()->bytes_left_to_send()) {
		//	send();
		//}
	}

	void send_byte(uint8_t byte) override {
		sendBuffer.push_back((char)byte);
	}

	//Handling RX
	uint8_t receive_byte() override { 
		uint8_t new_byte = receive_buffer.front();
		receive_buffer.pop_front();
		return new_byte;
	}

	//Misc

	// Accept windows comport
	orcaSDK::OrcaError open_serial_port(int, unsigned int) override {
		return { false, "" };
	}

	// Accept linux comport
	orcaSDK::OrcaError open_serial_port(std::string, unsigned int) override {
		return { false, "" };
	}

	void close_serial_port() {}
	void set_new_serial_port(int) {}

	void adjust_baud_rate(uint32_t new_baud_rate) override {
		adjusted_baud_rate = new_baud_rate;
	}

	bool ready_to_send() override {
		return true;
	}

	bool ready_to_receive() override {
		return !receive_buffer.empty();
	}


private:
	std::deque<char> receive_buffer;
	uint64_t current_time = 0;
};