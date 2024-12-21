/**
 * @file mb_config.h
    
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

#ifndef MB_CONFIG_H_
#define MB_CONFIG_H_

#define DEFAULT_INTERFRAME_uS	0	
#define DEFAULT_INTERCHAR_uS	16000
#define DEFAULT_TURNAROUND_uS	500		
#define DEFAULT_RESPONSE_uS    	8000	

//uncomment one of the following baud rate options
#define UART_BAUD_RATE      625000 //Modbus specified default is 19200bps

#endif
