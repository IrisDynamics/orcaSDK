#include "serial_interface.h"
#include "asio.hpp"
#include "error_types.h"
#include <deque>
#include <mutex>
#include <future>
#include <iostream>
#include <memory>

namespace orcaSDK {

class SerialASIO : public SerialInterface, public std::enable_shared_from_this<SerialASIO>
{
public:
	SerialASIO() :
		serial_port(io_context),
		work_guard(io_context.get_executor())
	{
		read_buffer.resize(256);
		io_context_run_thread = std::thread{ [=]() {
			io_context.run();
		} };
	}

	~SerialASIO()
	{
		close_serial_port();
		work_guard.reset();
		io_context_run_thread.join();
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
		std::lock_guard<std::mutex> lock{ write_lock };
		send_data.push_back(data);
	}

	void tx_enable(size_t _bytes_to_read) override {
		bytes_to_read = _bytes_to_read;
		serial_port.cancel();
		{
			//Clear any leftover read data
			std::lock_guard<std::mutex> lock{ read_lock };
			read_data.clear();
		}
		std::lock_guard<std::mutex> lock{ write_lock };
		asio::async_write(serial_port, asio::buffer(send_data), [me=shared_from_this()](const asio::error_code& ec, size_t bytes_written)
			{
				std::lock_guard<std::mutex> lock{ me->write_lock };
				me->send_data.clear();
				if (ec) return;
				me->read_message_function_code();
			});
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

	std::vector<uint8_t> receive_bytes_blocking() override
	{
		std::unique_lock<std::mutex> lock{ read_lock };
	
		if (read_data.size() < bytes_to_read)
		{
			//The wait time should be as small as possible, while being long
			// enough to ensure the response isn't going to arrive
			read_notifier.wait_for(lock, std::chrono::milliseconds(25)); 
		}
		
		std::vector<uint8_t> bytes_read = read_data;
		read_data.clear();
		return bytes_read;
	}

private:
	asio::io_context io_context;
	asio::serial_port serial_port;

	std::vector<uint8_t> send_data;
	std::vector<uint8_t> read_data;

	asio::executor_work_guard<asio::io_context::executor_type> work_guard;

	std::condition_variable read_notifier;

	std::mutex write_lock;
	std::mutex read_lock;

	std::thread io_context_run_thread;

	std::atomic<size_t> bytes_to_read{ 0 };

	std::vector<uint8_t> read_buffer;

	void read_message_function_code()
	{
		asio::async_read(serial_port,
			asio::buffer(read_buffer, 2), 
			[me = shared_from_this()](const asio::error_code& ec, size_t bytes_read) {
				if (ec || bytes_read != 2) return;
				if (me->read_buffer[1] & 0x80)
				{
					me->bytes_to_read = 5;
				}
				std::unique_lock<std::mutex> read_guard(me->read_lock);
				for (int i = 0; i < bytes_read; i++)
				{
					me->read_data.push_back(me->read_buffer[i]);
				}
				me->read_message_body();
			});	
	}

	void read_message_body()
	{
		asio::async_read(serial_port,
			asio::buffer(read_buffer.data() + 2, bytes_to_read - 2),
			[me = shared_from_this()](const asio::error_code& ec, size_t bytes_read)
			{
				if (ec) return;
				std::unique_lock<std::mutex> lock(me->read_lock);
				for (int i = 0; i < bytes_read; i++)
				{
					me->read_data.push_back(me->read_buffer[i+2]);
				}
				me->read_notifier.notify_one();
			});
	}
};

}
