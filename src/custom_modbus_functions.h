#pragma once

#include "transaction.h"
#include "message_priority.h"

namespace orcaSDK { namespace CustomModbusFunctions {

	Transaction manage_high_speed_stream(uint8_t device_address, uint32_t new_baud, uint16_t interframe_delay, MessagePriority priority) {
		constexpr int message_body_length = 8;
		constexpr int total_message_length = message_body_length + 4;
		
		uint8_t data_bytes[message_body_length] = { 0xff, 0, uint8_t(new_baud >> 24), uint8_t(new_baud >> 16), uint8_t(new_baud >> 8), uint8_t(new_baud), uint8_t(interframe_delay >> 8), uint8_t(interframe_delay)};
		Transaction my_temp_transaction;
		if (priority == MessagePriority::important) my_temp_transaction.mark_important();
		my_temp_transaction.load_transmission_data(
			device_address, 0x41, data_bytes, message_body_length,
			total_message_length);
		return my_temp_transaction;
	}

} }