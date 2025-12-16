#pragma once

#include "serial_interface.h"
#include "asio.hpp"
#include "error_types.h"
#include <deque>
#include <mutex>
#include <future>
#include <iostream>
#include <memory>

namespace orcaSDK {

class SerialASIO : public SerialInterface
{
public:
	SerialASIO()
	{
		read_buffer.resize(256);
		io_context_run_thread = std::thread{ [=]() {
			io_context.run();
		} };
	}

	~SerialASIO()
	{
		work_guard.reset();
		if (serial_port.is_open()) serial_port.cancel();
		io_context_run_thread.join();
		close_serial_port();
	}

	OrcaError open_serial_port(int serial_port_number, unsigned int baud) override
	{
#if defined _WIN32
		std::string port_name = std::string("\\\\.\\COM") + std::to_string(serial_port_number);
#else
		std::string port_name = std::string("/dev/ttyUSB") + std::to_string(serial_port_number);
#endif
		return open_serial_port(port_name, baud);
	}

	OrcaError open_serial_port(std::string serial_port_path, unsigned int baud) override
	{
		asio::error_code ec;
		serial_port.open(serial_port_path, ec);

		if (ec)	return { ec.value(), ec.message() };

		serial_port.set_option(asio::serial_port::baud_rate{ baud });
		serial_port.set_option(asio::serial_port::stop_bits{ asio::serial_port::stop_bits::type::one });
		serial_port.set_option(asio::serial_port::parity{ asio::serial_port::parity::type::even });

		return { 0 };
	}

	void close_serial_port() override {
		serial_port.close();
	}

	void adjust_baud_rate(uint32_t baud_rate_bps) override {
		if (serial_port.is_open()) serial_port.set_option(asio::serial_port::baud_rate{ baud_rate_bps });
	}

	bool ready_to_send() override {
		return true;
	}

	void send_byte(uint8_t data) override {
		send_data.push_back(data);
	}

	void tx_enable(size_t _bytes_to_read) override {
		if (!serial_port.is_open()) return;

		bytes_to_read = _bytes_to_read;
		serial_port.cancel();
		{
			//Clear any leftover read data
			std::lock_guard<std::mutex> lock{ read_lock };
			read_data.clear();
		}
		std::shared_ptr<std::vector<uint8_t>> write_buffer = std::make_shared<std::vector<uint8_t>>(send_data);
		// Copying write buffer in lambda capture list to keep it alive for the duration of the write
		asio::async_write(serial_port, asio::buffer(*write_buffer), [&, write_buffer](const asio::error_code& ec, size_t bytes_written)
			{
				if (ec) return;
				read_message_function_code();
			});
		send_data.clear();
	}

	bool ready_to_receive() override {
		std::lock_guard<std::mutex> lock{ read_lock };
		return read_data.size();
	}

	uint8_t receive_byte() override {
		std::lock_guard<std::mutex> lock{ read_lock };
		uint8_t byte = read_data.front();
		read_data.erase(read_data.begin(), read_data.begin() + 1);
		return byte;
	}

	OrcaResult<std::vector<uint8_t>> receive_bytes_blocking() override
	{
		if (!serial_port.is_open()) return { {}, { 1, "No serial port open." } };

		std::unique_lock<std::mutex> lock{ read_lock };
	
		if (read_data.size() < bytes_to_read)
		{
			//The wait time should be as small as possible, while being long
			// enough to ensure the response isn't going to arrive
			timeout_occurred = true;
			read_notifier.wait_for(lock, std::chrono::milliseconds(25)); 
		}
		
		std::vector<uint8_t> bytes_read = read_data;
		read_data.clear();

		if (timeout_occurred)
		{
			return { bytes_read, {1, "Read blocking timed out."} };
		}
		else
		{
			return { bytes_read, {0, ""} };
		}
	}

	void flush_and_discard_receive_buffer() override {
		if (!serial_port.is_open()) return;

		std::array<uint8_t, 256> flush_buffer;
		asio::async_read(serial_port, asio::buffer(flush_buffer, 256), [&](asio::error_code ec, size_t bytes_read) {
			// Need a completion handler, even if empty, for the read to execute
			});

		std::promise<bool> wait_barrier;
		std::future<bool> wait_barrier_future = wait_barrier.get_future();

		asio::steady_timer t{io_context, std::chrono::milliseconds(20)};
		t.async_wait([&](asio::error_code ec) {
			serial_port.cancel();
			wait_barrier.set_value(true);
			});

		wait_barrier_future.get();
	}
	
	bool is_open() override {
		return serial_port.is_open();
	}

private:
	asio::io_context io_context;
	asio::serial_port serial_port{io_context};
	asio::executor_work_guard<asio::io_context::executor_type> work_guard{asio::make_work_guard(io_context)};

	std::vector<uint8_t> send_data;
	std::vector<uint8_t> read_data;

	std::atomic<bool> timeout_occurred{false};
	std::condition_variable read_notifier;

	std::mutex read_lock;

	std::thread io_context_run_thread;

	std::atomic<size_t> bytes_to_read{ 0 };

	std::vector<uint8_t> read_buffer;

	void read_message_function_code()
	{
		asio::async_read(serial_port,
			asio::buffer(read_buffer, 2), 
			[&](const asio::error_code& ec, size_t bytes_read) {
				if (ec || bytes_read != 2) return;
				if (read_buffer[1] & 0x80)
				{
					bytes_to_read = 5;
				}
				std::unique_lock<std::mutex> read_guard(read_lock);
				for (int i = 0; i < bytes_read; i++)
				{
					read_data.push_back(read_buffer[i]);
				}
				read_message_body();
			});	
	}

	void read_message_body()
	{
		asio::async_read(serial_port,
			asio::buffer(read_buffer.data() + 2, bytes_to_read - 2),
			[&](const asio::error_code& ec, size_t bytes_read)
			{
				if (ec) return;
				std::unique_lock<std::mutex> lock(read_lock);
				for (int i = 0; i < bytes_read; i++)
				{
					read_data.push_back(read_buffer[i+2]);
				}
				timeout_occurred = false;
				read_notifier.notify_one();
			});
	}
};

}
