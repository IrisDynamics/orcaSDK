#pragma once

#include <cstdint>

class SerialInterface {
public:
	virtual void init(int baud) = 0;

	virtual void adjust_baud_rate(uint32_t baud_rate_bps) = 0;

	virtual bool ready_to_send() = 0;

	virtual void send_byte(uint8_t data) = 0;

	virtual void tx_enable() = 0;

	virtual bool ready_to_receive() = 0;

	virtual uint8_t receive_byte() = 0;
};