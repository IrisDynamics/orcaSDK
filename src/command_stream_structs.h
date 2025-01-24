#pragma once

namespace orcaSDK
{
	/**
	 *	@brief	The struct for storing data returned from the Orca in responses to command stream
	 *			messages.
	 */
	struct StreamData
	{
		int32_t position{ 0 }; //!<The position in micrometers from the zero position of the motor.
		int32_t force{ 0 }; //!<The sensed force of the motor.
		uint16_t power{ 0 }; //!<The power in Watts that the motor detects/is consuming.
		int16_t temperature{ 0 }; //!<The sensed temperature of the motor's microcontroller, in celsius.
		uint16_t voltage{ 0 }; //!<The sensed voltage of the motor's connected power supply, in millivolts.
		uint16_t errors{ 0 }; //!<Any active errors present on the motor.
	};
}