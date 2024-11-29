#include "standard_modbus_functions.h"
#include <stdint.h>
#include "function_code_parameters.h"
#include "modbus_client.h"
#include "message_priority.h"

namespace DefaultModbusFunctions {

	/**
	 * @brief Format a read_holding_registers request, function code 03, and add the request to the buffer queue
	 * @param device_address The address of the server device that will accept the transaction, 0 if broadcast
	 * @param starting_address The address of the first holding register to start reading from
	 * @param num_registers The quanity of holding registers to read
	 * @return An integer - 1 if the transaction is formatted and added to the buffer queue successfuly, 0 if an exception occurs
	 */
	Transaction read_holding_registers_fn(uint8_t device_address, uint16_t starting_address, uint16_t num_registers, MessagePriority priority) {
		uint8_t data_bytes[4] = { uint8_t(starting_address >> 8), uint8_t(starting_address), uint8_t(num_registers >> 8), uint8_t(num_registers) };
		Transaction my_temp_transaction;
		if (priority == MessagePriority::important) my_temp_transaction.mark_important();
		my_temp_transaction.load_transmission_data(
			device_address, ModbusFunctionCodes::read_holding_registers, data_bytes, 4,
			5 + (num_registers * 2));
		return my_temp_transaction;
	}

	/**
	 * @brief Format a write_single_register request, function code 06, and add the request to the buffer queue
	 * @param device_address The address of the server device that will accept the transaction, 0 if broadcast
	 * @param address The address of the register to write to
	 * @param data The value to write to the register
	 * @return An integer - 1 if the transaction is formatted and added to the buffer queue successfuly, 0 if an exception occurs
	 */
	Transaction write_single_register_fn(uint8_t device_address, uint16_t address, uint16_t data, MessagePriority priority) {
		uint8_t data_bytes[4] = { uint8_t(address >> 8), uint8_t(address), uint8_t(data >> 8), uint8_t(data) };
		Transaction my_temp_transaction;
		if (priority == MessagePriority::important) my_temp_transaction.mark_important();
		my_temp_transaction.load_transmission_data(
			device_address, ModbusFunctionCodes::write_single_register, data_bytes, 4,
			WRITE_OR_GET_COUNTER_RESPONSE_LEN);
		return my_temp_transaction;
	}

	/**
	 * @brief Format a write_multiple_registers request, function code 16, and add the request to the buffer queue
	 * @param device_address The address of the server device that will accept the transaction, 0 if broadcast
	 * @param starting_address The address of the first register to start writing to
	 * @param num_registers The quantity of registers to write to
	 * @param data An array of data that will be written, in order, to the registers beginning at starting_address
	 * @return An integer, 1 if the transaction is formatted and added to the buffer queue successfuly, 0 if an exception occurs
	 */
	Transaction write_multiple_registers_fn(uint8_t device_address, uint16_t starting_address, uint16_t num_registers, uint8_t* data, MessagePriority priority) {
		uint8_t num_bytes = uint8_t(num_registers) * 2;
		uint8_t data_bytes[5] = { uint8_t(starting_address >> 8),
											  uint8_t(starting_address),
											  uint8_t(num_registers >> 8),
											  uint8_t(num_registers),
											  num_bytes };
		Transaction my_temp_transaction;
		if (priority == MessagePriority::important) my_temp_transaction.mark_important();
		my_temp_transaction.load_transmission_data(
			device_address, ModbusFunctionCodes::write_multiple_registers, data_bytes, 5, data, num_bytes,
			WRITE_OR_GET_COUNTER_RESPONSE_LEN);
		return my_temp_transaction;
	}

	/**
	 * @brief Format a read_write_multiple_registers request, function code 23, and add the request to the buffer queue
	 * @param device_address The address of the server device that will accept the transaction, 0 if broadcast
	 * @param read_starting_address The address of the first register to start reading from
	 * @param read_num_registers The quantity of registers to read from
	 * @param write_starting_address The address of the first register to write to
	 * @param write_num_registers The quantity of registers to write to
	 * @param data An array of data that will be written, in order, to the register(s) beginning at write_start_address
	 * @return An integer - 1 if the transaction is formatted and added to the buffer queue successfuly, 0 if an exception occurs
	 */
	Transaction read_write_multiple_registers_fn(uint8_t device_address,
		uint16_t read_starting_address, uint16_t read_num_registers,
		uint16_t write_starting_address, uint16_t write_num_registers,
		uint8_t* data, MessagePriority priority)
	{
		uint8_t write_num_bytes = uint8_t(write_num_registers) * 2;

		uint8_t data_bytes[9] = { uint8_t(read_starting_address >> 8),
													uint8_t(read_starting_address),
													uint8_t(read_num_registers >> 8),
													uint8_t(read_num_registers),
													uint8_t(write_starting_address >> 8),
													uint8_t(write_starting_address),
													uint8_t(write_num_registers >> 8),
													uint8_t(write_num_registers),
													write_num_bytes };
		//for(int i  = 0; i < write_num_bytes; i++) data_bytes[i + 9] = data[i];
		Transaction my_temp_transaction;
		if (priority == MessagePriority::important) my_temp_transaction.mark_important();
		my_temp_transaction.load_transmission_data(
			device_address, ModbusFunctionCodes::read_write_multiple_registers, data_bytes, 9, data, write_num_bytes,
			5 + read_num_registers * 2);
		return my_temp_transaction;

	}
};