#pragma once

#include <cstdint>

/**
 *	@brief	The pure virtual interface that the SDK relies on for performing
 *			serial communication. Must be implemented if using the SDK on 
 *			unsupported platforms.
 */
class SerialInterface {
public:
	virtual ~SerialInterface() = default;

	/**
	 *	@brief	Obtains the currently indicate serial port. Setting baud rate
	 *			based on the passed parameter.
	 */
	virtual void init(int baud) = 0;

	/**
	 *	@brief	Changes the current baud rate of the serial port.
	 */
	virtual void adjust_baud_rate(uint32_t baud_rate_bps) = 0;

	/**
	 *	@brief	Returns true if there isn't currently a write in progress on
	 *			this serial port.
	 */
	virtual bool ready_to_send() = 0;

	/**
	 *	@brief	Queues a byte to be sent by the next call to tx_enable()
	 */
	virtual void send_byte(uint8_t data) = 0;

	/**
	 *	@brief	Writes all currently queued/buffered bytes to the serial port.
	 */
	virtual void tx_enable() = 0;

	/**
	 *	@brief	Returns true if there is received data ready to be consumed by
	 *			the SDK.
	 */
	virtual bool ready_to_receive() = 0;

	/**
	 *	@brief	Pops and returns a byte from the received buffer.
	 */
	virtual uint8_t receive_byte() = 0;
};