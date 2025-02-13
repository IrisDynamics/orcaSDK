#include "serial_interface.h"
#include "asio.hpp"
#include "error_types.h"
#include <deque>
#include <mutex>
#include <future>
#include <iostream>

namespace orcaSDK {

class SerialASIO : public SerialInterface
{
public:
	SerialASIO() :
		serial_port(io_context)
	{}

	~SerialASIO()
	{
		close_serial_port();
	}

	OrcaError open_serial_port(int serial_port_number, unsigned int baud) override
	{
#if defined _WIN32
		std::string port_name = std::string("\\\\.\\COM") + std::to_string(serial_port_number);
#else
		std::string port_name = std::string("/dev/ttyS") + std::to_string(serial_port_number);
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

		work_to_do = true;

		read_thread = std::thread{ [=]() {
			read_incoming_data();
		} }; 

		//work_thread = std::thread{ [=]() {
		//	io_context.run();
		//} };

		return { 0 };
	}

	void close_serial_port() override {
		work_to_do = false;
		//work_guard.reset();
		serial_port.cancel();
		serial_port.close();
		io_context.run();
		read_thread.join();
		//work_thread.join();
	}

	void adjust_baud_rate(uint32_t baud_rate_bps) override {
		serial_port.set_option(asio::serial_port::baud_rate{ baud_rate_bps });
	}

	bool ready_to_send() override {
		return true;
	}

	void send_byte(uint8_t data) override {
		//std::lock_guard<std::mutex> lock_gd{ write_lock };
		send_data.push_back(data);
	}

	void tx_enable(size_t _bytes_to_read) override {
		bytes_to_read = _bytes_to_read;
		size_t num_sent = serial_port.write_some(asio::buffer(send_data));
		send_data.erase(send_data.begin(), send_data.begin() + num_sent);
		//serial_port.async_write_some(asio::buffer(send_data), [this](const asio::error_code& ec, size_t num_sent)
		//	{
		//		std::lock_guard<std::mutex> lock_gd{ write_lock };
		//		send_data.erase(send_data.begin(), send_data.begin() + num_sent);
		//	});
	}

	bool ready_to_receive() override {
		std::lock_guard<std::mutex> light_lock{read_light_lock};
		std::lock_guard<std::mutex> lock{ read_lock };
		return read_data.size();
	}

	uint8_t receive_byte() override {
		std::lock_guard<std::mutex> light_lock{ read_light_lock };
		std::lock_guard<std::mutex> lock{ read_lock };
		uint8_t byte = read_data.front();
		read_data.erase(read_data.begin(), read_data.begin() + 1);
		return byte;
	}

private:
	asio::io_context io_context;
	asio::serial_port serial_port;

	std::vector<uint8_t> send_data;
	std::vector<uint8_t> read_data;

	std::atomic<bool> work_to_do{ false };
	//asio::executor_work_guard<asio::io_context::executor_type> work_guard;

	//std::mutex write_lock;

	std::mutex read_lock;
	std::mutex read_light_lock;

	std::thread read_thread;
	//std::thread work_thread;

	std::atomic<size_t> bytes_to_read{ 0 };

	void read_incoming_data()
	{
		std::vector<uint8_t> read_buffer;
		read_buffer.resize(256);
//		asio::cancellation_signal cancel_signal;
//		asio::steady_timer timer{io_context};
//
//using std::chrono::steady_clock;
//using std::chrono::microseconds;
		while (work_to_do)
		{
			asio::error_code ec1;
			size_t bytes_read = serial_port.read_some(asio::buffer(read_buffer, 2), ec1);

			if (ec1)
			{
				//std::cerr << "Read Function Code Failed" << ec1.message() << ". Bytes read: " << bytes_read << "\n";
				continue;
			}

			if (read_buffer[1] & 0x80)
			{
				//Error code encountered
				bytes_to_read = 5;
			}

			asio::error_code ec2;
			bytes_read = serial_port.read_some(asio::buffer(read_buffer.data() + 2, bytes_to_read - 2), ec2);

			if (ec2)
			{
				std::cerr << "Read Remaining Message Failed" << ec2.message() << "\n";
				continue;
			}
			
			std::unique_lock<std::mutex> lock{ read_lock };

			for (int i = 0; i < bytes_read + 2; i++)
			{
				read_data.push_back(read_buffer[i]);
			}

			lock.unlock();
			std::lock_guard<std::mutex> light_lock{ read_light_lock };
			//auto time_start = steady_clock::now();

			//timer.expires_after((steady_clock::now() + microseconds(500)) - steady_clock::now());
			//timer.async_wait([&cancel_signal, &time_start](const asio::error_code) {
			//	cancel_signal.emit(asio::cancellation_type::partial);
			//	std::cout << "Timer emit after " << (steady_clock::now() - time_start).count() << "\n";
			//	});
			/*asio::async_read(serial_port, asio::buffer(read_buffer), asio::bind_cancellation_slot(
				cancel_signal.slot(),
				[&read_buffer, this](const asio::error_code& ec, size_t amount_read)
				{
					if (ec.value() != 995)
					{
						std::cout << "Read error: " << ec.message() << std::endl;
						return;
					}

					std::unique_lock<std::mutex> lock{ read_lock };

					for (int i = 0; i < amount_read; i++)
					{
						read_data.push_back(read_buffer[i]);
					}

					lock.unlock();
					std::lock_guard<std::mutex> light_lock{ read_light_lock };

					std::cout << "Read Iteration Complete\n";
				}
				));*/

			//timer.wait();
		}
	}
};

}