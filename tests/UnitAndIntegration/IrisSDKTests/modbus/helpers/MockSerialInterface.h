#pragma once

#include "src/serial_interface.h"
#include <gmock/gmock.h>

class MockSerialInterface : public SerialInterface
{
public:
	MOCK_METHOD(void, init, (int baud), (override));
	MOCK_METHOD(void, adjust_baud_rate, (uint32_t baud_rate_bps), (override));
	MOCK_METHOD(bool, ready_to_send, (), (override));
	MOCK_METHOD(void, send_byte, (uint8_t data), (override));
	MOCK_METHOD(void, tx_enable, (), (override));
	MOCK_METHOD(bool, ready_to_receive, (), (override));
	MOCK_METHOD(uint8_t, receive_byte, (), (override));
};