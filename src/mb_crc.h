/**
 *  @file mb_crc.h
 *
 *  Created on: Aug. 3, 2021
 *  @author: Sean
    
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

#ifndef MB_CRC_H_
#define MB_CRC_H_

#include <stdint.h>
#include "constants.h"

namespace orcaSDK
{

/**
 * @class ModbusCRC
 * @brief For generating a 16 bit CRC in accordance with the Modbus specification.
 */
 class ModbusCRC {
public:

	/**
	 * @brief Generates and returns a 16 bit CRC for a given message.
	 * 	      The return CRC already has the byte order swapped and is ready to be placed in a Modbus message.
	 *
	 * @param	message			Pointer to the message buffer to be used for CRC generation.
	 * @param 	message_len	 	Number of bytes in the message buffer.
	 */
	static constexpr uint16_t generate(uint8_t *message, int message_len) {

		uint8_t crc_hi_byte = 0xFF;	// initialize crc bytes
		uint8_t crc_lo_byte = 0xFF; //
		int index = 0; // for indexing the crc tables

		while(message_len--) {

			// calculate crc
			index = crc_hi_byte ^ *message++;
			crc_hi_byte = crc_lo_byte ^ Constants::crc_hi_table[index];
			crc_lo_byte = Constants::crc_lo_table[index];

		}

		return (crc_hi_byte << 8 | crc_lo_byte);	// return crc result, with bytes swapped for modbus message

	}

};

}

#endif /* MB_CRC_H_ */
