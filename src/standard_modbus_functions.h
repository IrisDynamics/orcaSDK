/**
 * @file modbus_client_application.h
 * @author Kali Erickson <kerickson@irisdynamics.com>
 * 
 * @brief  Base implementation of a Modbus client application layer
 * 
 * @version 2.2.0
    
    @copyright Copyright 2022 Iris Dynamics Ltd 
    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.

    For questions or feedback on this file, please email <support@irisdynamics.com>. 
 */

#ifndef MODBUS_CLIENT_APPLICATION_H_
#define MODBUS_CLIENT_APPLICATION_H_

#include <stdint.h>
#include "function_code_parameters.h"
#include "message_priority.h"
#include "transaction.h"

namespace orcaSDK { 

namespace ModbusSubFunctionCodes
{
	/**
	 * @brief Enum of all supported diagnostic sub-function codes.
	 */
	enum sub_function_codes_e {
		return_query_data = 00,
		restart_communications = 01,
		return_diagnostic_register = 02,
		force_listen_only_mode = 04,
		clear_counters_and_diagnostic_register = 10,
		return_bus_message_count = 11,
		return_bus_communication_error_count = 12,
		return_bus_exception_error_count = 13,
		return_server_message_count = 14,
		return_server_no_response_count = 15,
		return_server_NAK_count = 16,
		return_server_busy_count = 17,
		return_bus_character_overrun_count = 18,
		clear_overrun_counter_and_flag = 20
	};
}

namespace ModbusFunctionCodes
{
	/**
	 * @brief Enum of all supported function codes.
	 */
	enum function_codes_e {
		read_coils = 0x01,
		read_discrete_inputs = 0x02,
		read_holding_registers = 0x03,
		read_input_registers = 0x04,
		write_single_coil = 0x05,
		write_single_register = 0x06,
		read_exception_status = 0x07,
		diagnostics = 0x08,
		get_comm_event_counter = 0x0B,
		get_comm_event_log = 0x0C,
		write_multiple_coils = 0x0F,
		write_multiple_registers = 0x10,
		report_server_id = 0x11,
		mask_write_register = 0x16,
		read_write_multiple_registers = 0x17
	};
}

/**
 * @class ModbusClientApplication
 * @brief Base implementation of a Modbus client application layer.
 *
 * To add a function code in a derived class:
 *
 * 1. Add the function code's name and code number (in hex) to a public function code enum in the derived class.
 *
 * 2. Add the expected length of the function code's response to the function_code_parameters.h file if it is known.
 *    If the length is variable, don't define it. Instead, calculate it in the implementation of the get_app_reception_length function.
 *
 * 	  Other useful definitions pertaining to the function code may be added here as well, such as valid data ranges.
 *
 * 3. Add a case returning the expected length to the implementation of the get_app_response_length function.
 *    If the length is unknown until response reception, return -1.
 *
 * 4. Add a private function ( "function_code_name_fn()" ) to the derived class that loads transmission with the properly formatted message and then calls send_transaction.
 *    Return a 1 if the function is successful, return a 0 if an exception occurs.
 */

namespace DefaultModbusFunctions {

	/**
	 * @brief Format a read_holding_registers request, function code 03, and add the request to the buffer queue
	 * @param device_address The address of the server device that will accept the transaction, 0 if broadcast
     * @param starting_address The address of the first holding register to start reading from
     * @param num_registers The quanity of holding registers to read
	 * @return An integer - 1 if the transaction is formatted and added to the buffer queue successfuly, 0 if an exception occurs
	 */
	Transaction read_holding_registers_fn(uint8_t device_address, uint16_t starting_address, uint16_t num_registers, MessagePriority priority);

    /**
	 * @brief Format a write_single_register request, function code 06, and add the request to the buffer queue
	 * @param device_address The address of the server device that will accept the transaction, 0 if broadcast
     * @param address The address of the register to write to
     * @param data The value to write to the register
	 * @return An integer - 1 if the transaction is formatted and added to the buffer queue successfuly, 0 if an exception occurs
	 */
	Transaction write_single_register_fn(uint8_t device_address, uint16_t address, uint16_t data, MessagePriority priority);

	/**
	 * @brief Format a write_multiple_registers request, function code 16, and add the request to the buffer queue
	 * @param device_address The address of the server device that will accept the transaction, 0 if broadcast
     * @param starting_address The address of the first register to start writing to
     * @param num_registers The quantity of registers to write to
     * @param data An array of data that will be written, in order, to the registers beginning at starting_address
	 * @return An integer, 1 if the transaction is formatted and added to the buffer queue successfuly, 0 if an exception occurs
	 */
	Transaction write_multiple_registers_fn(uint8_t device_address, uint16_t starting_address, uint16_t num_registers, uint8_t* data, MessagePriority priority);

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
		uint8_t* data, MessagePriority priority);
}

}

#endif /* MODBUS_APPLICATION_H_ */