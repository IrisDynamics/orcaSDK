#pragma once
#include <deque>
#include "modbus_client/src/mb_crc.h"

namespace ModbusTesting
{
	void CalculateAndAppendCRC(std::deque<char>& modbus_message)
	{
		uint8_t array_of_bytes[256];
		for (int i = 0; i < modbus_message.size(); i++)
		{
			array_of_bytes[i] = modbus_message[i];
		}
		uint16_t crc = ModbusCRC::generate(array_of_bytes, modbus_message.size());
		
		char high_byte = crc >> 8;
		char low_byte = crc;
		modbus_message.push_back(high_byte);
		modbus_message.push_back(low_byte);
	}
}