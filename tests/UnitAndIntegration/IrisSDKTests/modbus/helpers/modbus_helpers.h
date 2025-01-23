#pragma once
#include <deque>
#include "src/mb_crc.h"

namespace ModbusTesting
{
	static void CalculateAndAppendCRC(std::deque<char>& modbus_message)
	{
		uint8_t array_of_bytes[256];
		for (int i = 0; i < modbus_message.size(); i++)
		{
			array_of_bytes[i] = modbus_message[i];
		}
		uint16_t crc = orcaSDK::ModbusCRC::generate(array_of_bytes, (int)modbus_message.size());
		
		char high_byte = char(crc >> 8);
		char low_byte = char(crc);
		modbus_message.push_back(high_byte);
		modbus_message.push_back(low_byte);
	}
}