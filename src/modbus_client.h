/**
 * @file modbus_client.h
 * @author Kali Erickson <kerickson@irisdynamics.com>
 * 
 * @brief Modbus client serial communication virtual class
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

#ifndef MODBUS_CLIENT_H_
#define MODBUS_CLIENT_H_

#include "serial_interface.h"
#include "diagnostics_tracker.h"
#include "message_queue.h"
#include "../tools/log_interface.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include "clock.h"
#include <memory>

namespace orcaSDK
{

/**
 * @class ModbusClient
 * @brief Modbus client serial communication virtual class. To be implemented by device drivers.
 * 
 * This class uses a message buffer queue object, send/receive and timer dependent functions to be called by a derived class, and performs message validation.
 *
 * @section CLIENT_TIMER Client Timer
 *
 * The client uses a single timer to observe pauses in transmission, or determine when timeouts have occurred.
 * Only one delay timer / timeout timer can be active at once.
 * The timers present are:
 * # Response timeout - starts after all bytes of a message are sent to the transmitter, is cleared by receiving a byte, and expiry invalidates a message.
 * # Intercharacter timeout - starts after receiving a byte in the receiving state, is cleared when receiving the message's known payload, and expiry invalidates messages of known size, and triggers validation of unknown-size messages
 * # Interframe delay - starts following validation/invalidation of a message, is cleared only when it expires, and expiry returns the client to Idle
 * # Broadcast delay - starts following transmission of a broadcast message, is cleared when expiring and expiry enables the interframe delay
 *
 * */
class ModbusClient {
public:

    /**
    * @brief construct ModbusClient object
    * @param _channel_number specify channel number, particularly relevant if there are multiple uart channels possible
    */                       
    ModbusClient(
        SerialInterface& serial_interface,
        Clock& clock
    ):
        serial_interface(serial_interface),
        clock(clock),
		repsonse_timeout_cycles  ( Constants::kDefaultResponseTimeout_uS	 ),	
		interchar_timeout_cycles ( Constants::kDefaultIntercharTimeout_uS	 ),
		turnaround_delay_cycles  ( Constants::kDefaultTurnaroundTime_uS      )
    {}
    virtual ~ModbusClient(){}

    /**
     * @brief a debugging function which prints the state of the client
     */
    void printme() {
#ifdef IRISCONTROLS
	    PRINTDL( "timer enabled: " 	, my_enabled_timer);
        PRINTDL( "modbus state: "	, my_state );
		PRINTDL( "queue size: "		, get_queue_size());
    	messages.printme();
#endif
    }

    /**
      * @brief init tranceiver hardware
     */
    void init(int interframe_delay_us)
    {
        reset_state();
        interframe_delay_cycles = interframe_delay_us;
    }

    /**
     * @brief brings the state machine back to an initial state
     */
    void reset_state () {
    	messages.reset();
    	enable_interframe_delay();
    }


    /**
     * @brief used to deactivate messages on the queue, poll the receiver hardware for buffered data, observe timeout timers
     * Should be polled at least as fast as messages are expected to arrive
     */
    void run_in() {
    	// Looking for message parsing? Check the application run_in() function (e.g. Actuator object).
        receive();

    	// No timers are enabled
    	if ( my_enabled_timer == TIMER_ID::none ) {

    	}
    	// A timer is enabled
    	else {
            Transaction* active_transaction;


			TIMER_ID expired_timer = has_timer_expired();
			switch (expired_timer) {

			case TIMER_ID::repsonse_timeout:
                active_transaction = messages.get_active_transaction();
                diagnostic_counters.increment_diagnostic_counter(return_server_no_response_count);
				active_transaction->invalidate(Transaction::RESPONSE_TIMEOUT_ERROR);
                conclude_transaction(active_transaction);
				break;

			case TIMER_ID::interchar_timeout:
                active_transaction = messages.get_active_transaction();


				// If the length was unknown assume this was the expected termination of the response until it is validated
                // TODO: Remove this once we have good test coverage of different modbus messages. I have reason to believe this never evaluates to true in a system without bugs
				if( !active_transaction->is_expected_length_known() ){ 
					active_transaction->validate_response(diagnostic_counters);
				}
				// If the length was known and an interchar timeout occurred (ie the message got messed up)
				else {
                    diagnostic_counters.increment_diagnostic_counter(unexpected_interchar);
					active_transaction->invalidate(Transaction::INTERCHAR_TIMEOUT_ERROR);
                    diagnostic_counters.increment_diagnostic_counter(ignoring_state_error);
				}

                conclude_transaction(active_transaction);
				break;


			case TIMER_ID::turnaround_delay:
				enable_interframe_delay();
				break;


			case TIMER_ID::interframe_delay:	// handled by run_out
			case TIMER_ID::none:				// No timers have expired, but one is enabled
			default:
				break;
			}

    	}

    }

    /**
     * @brief used to advance the message queue, start transmission of a new message, and observe interframe delay
     * Enables the response timer and starts the transmitter.
     * Should be polled at least as fast as messages are intended to be dispatched
     */
    void run_out () {
       
    	// If the interframe has expired, or there are no timers expired, check for a message to transmit
    	if ( my_enabled_timer == TIMER_ID::none	|| has_timer_expired() == TIMER_ID::interframe_delay) {
            if (has_timer_expired() == TIMER_ID::interframe_delay) flush_remaining_bytes_from_serial_interface();
    		disable_timer();
    		if ( messages.available_to_send() ) {
                if (serial_interface.ready_to_send())
                {
                    send_front_message();
                }
    		}
    	}
    }

    void send_front_message()
    {
        //while there are bytes left to send in the transaction, continue adding them to sendBuf
        Transaction* active_transaction = messages.get_active_transaction();

        if (!active_transaction->is_queued()) return;

        while (active_transaction->bytes_left_to_send()) {
            //send the current data byte
            uint8_t data = active_transaction->pop_tx_buffer();
            serial_interface.send_byte(data);
            diagnostic_counters.increment_diagnostic_counter(bytes_out_count);

            if (active_transaction->is_fully_sent()) {

                if (active_transaction->is_broadcast_message()) { //is it a broadcast message?
                    enable_turnaround_delay();
                }
                else {
                    enable_response_timeout();
                }
            }
        }
        if (logging) log_transaction_transmission(active_transaction);

        messages.mark_active_message_sent();
        serial_interface.tx_enable(active_transaction->get_expected_length());		// enabling the transmitter interrupts results in the send() function being called until the active message is fully sent to hardware
        diagnostic_counters.increment_diagnostic_counter(message_sent_count);    //temp? - for frequency benchmarking

    }

    void receive_blocking() {
        Transaction* active_transaction = messages.get_active_transaction();

        if (serial_interface.is_open())
        {
            OrcaResult<std::vector<uint8_t>> response_read = serial_interface.receive_bytes_blocking();

            if (response_read.error)
            {
                active_transaction->invalidate(Transaction::RESPONSE_TIMEOUT_ERROR);
                active_transaction->mark_finished();
            }
            else
            {
                std::vector<uint8_t> response = response_read.value;

                for (int i = 0; i < response.size(); i++)
                {
                    active_transaction->load_reception(response[i]); //read the next byte from the receiver buffer. This clears the byte received interrupt    ??TODO: should we be loading here? it seems that in the overrun case we've already walked off the end of the array??
                    diagnostic_counters.increment_diagnostic_counter(bytes_in_count);
                }

                active_transaction->validate_response(diagnostic_counters);// might transition to resting from connected
            }
        }
        else
        {
            active_transaction->invalidate(Transaction::SERIAL_PORT_NOT_OPEN);
            active_transaction->mark_finished();
        }

        if (!active_transaction->is_reception_valid())
        {
            serial_interface.flush_and_discard_receive_buffer();
        }
        conclude_transaction(active_transaction);
    }

////////////////////////////////////////////////////////////
///////////////////////// Queuing and Dequeuing Messages //
//////////////////////////////////////////////////////////

    /**
     * @brief enqueue a Transaction
     * @param message should be a populated Transaction object which will be copied into a Transaction in the message queue
    */
    void enqueue_transaction(Transaction message) {       
        messages.enqueue(message);
    }

    /**
     * @brief Determine if the message at the front of the queue is ready to be claimed - ie has received a response or encountered an error
     * @return true if the message ready to be claimed
    */
    bool is_response_ready(){
        return messages.is_response_ready();
    }

    /**
     * @brief dequeue a transaction from the message queue
     * @return 0 when dequeue fails, or the address of the dequeued message otherwise
    */
    Transaction dequeue_transaction(){
        return messages.dequeue();
    }

    /**
    * @brief get number of messages in the queue
    * @return True if the queue is empty (has no messages), False otherwise.
    */
    size_t get_queue_size(){
        return messages.size();
    }


/////////////////////////////////////////////////////////////
///////////////////////////////// Configuration Functions //
///////////////////////////////////////////////////////////

    /**
     * @brief Reconfigure the interframe delay in clock cycles
     * This is the minimum time that will be observed after receiving a message before a new message is transmitted
     * @param delay the minimum idle time between frames, in microseconds.
    */
    void adjust_interframe_delay_us(uint32_t delay) {
    	interframe_delay_cycles = delay;
    }

    /**
     * @brief Adjust the baud rate
     * @param baud_rate the new baud rate in bps
    */
    virtual void adjust_baud_rate(uint32_t baud_rate) {
        serial_interface.adjust_baud_rate(baud_rate);
    }


    /// @brief Change the time required to elapse before a message is deemed failed. Used to reduce from the default after a handshake negotiates a higher baud
    void adjust_response_timeout    (uint32_t time_in_us) { 	repsonse_timeout_cycles = time_in_us; };
    /// @brief Change the time required to elapse between characters within a message before it is abandoned.
	void adjust_interchar_timeout   (uint32_t time_in_us) { interchar_timeout_cycles = time_in_us; };
	/// @brief Change the period of time observed between broadcast messages
	void adjust_turnaround_delay	(uint32_t time_in_us) { 	turnaround_delay_cycles = time_in_us; };

    /**
     * @brief Get the device's current system time in cycles
    */
    int64_t get_system_cycles() {
        return clock.get_time_microseconds();
    }

    void begin_logging(std::shared_ptr<LogInterface> _log)
    {
        log = _log;
        logging = true;
    }

    DiagnosticsTracker diagnostic_counters;

private:
    SerialInterface& serial_interface;
    Clock& clock;

    std::shared_ptr<LogInterface> log;

    MessageQueue messages{ diagnostic_counters };            //!<a buffer for outgoing messages to facilitate timing and order of transmissions and responses

    int64_t repsonse_timeout_cycles;
    int64_t interchar_timeout_cycles;
    int64_t turnaround_delay_cycles;

    int64_t interframe_delay_cycles{ Constants::kDefaultInterframeDelay_uS };

    bool logging = false;

    /// Time that the enabled timer was started


    int64_t timer_start_time;	// recorded in system cycles: must be checked as such
    /**
     * @brief Should be run when ready to send a new byte.
     *	Transitions to reception when done sending.
     */
    void send(){

		Transaction * active_transaction = messages.get_active_transaction();

		//send the current data byte
		uint8_t data = active_transaction->pop_tx_buffer();
		serial_interface.send_byte(data);
        diagnostic_counters.increment_diagnostic_counter(bytes_out_count);

		if ( active_transaction->is_fully_sent() ) {

			if ( active_transaction->is_broadcast_message() ){ //is it a broadcast message?
				enable_turnaround_delay();
			}else{
				enable_response_timeout();
			}
		}
    }

    /**
	 * @brief Should only be run when a new byte has been received.
	 * 		  Example: Call from UART byte received interrupt or when polling the hardware for data in the input fifo
	 */
	void receive() {
        if (messages.size() == 0) return;
        
        Transaction* active_transaction = messages.get_active_transaction();

        if (!active_transaction->is_active()) return;

        while (serial_interface.ready_to_receive())
        {
            uint8_t byte = serial_interface.receive_byte();
            active_transaction->load_reception(byte); //read the next byte from the receiver buffer. This clears the byte received interrupt    ??TODO: should we be loading here? it seems that in the overrun case we've already walked off the end of the array??
            diagnostic_counters.increment_diagnostic_counter(bytes_in_count);

            // If this was the last character for this message
            if (active_transaction->received_expected_number_of_bytes())
            {
                active_transaction->validate_response(diagnostic_counters);// might transition to resting from connected
                conclude_transaction(active_transaction);
                break;
            }
            else {
                enable_interchar_timeout();
            }
        }
    }


/////////////////////////////////////////////////////////////
///////////////////////////////// The Timer ////////////////
///////////////////////////////////////////////////////////

	/**
	 * @brief represents which timer is active (ie only one timer is present at any given time)
	 */
	enum TIMER_ID {
		none,
		repsonse_timeout,
		interchar_timeout,
		turnaround_delay,
		interframe_delay

	};

	TIMER_ID my_enabled_timer = none;

    /**
     * @brief Start/enable the interframe delay timer.
     *        The interframe delay is the minimum idle time between message frames
     *        To ensure accuracy, an interrupt based timer should be used if it is available
    */
    void enable_interframe_delay() {
    	timer_start_time = get_system_cycles();
    	my_enabled_timer = TIMER_ID::interframe_delay;
    }

    /**
     * @brief Start/enable the intercharacter timeout timer.
     *        The intercharacter timeout is the maximum time between characters in a message frame
    */
    void enable_interchar_timeout() {
    	timer_start_time = get_system_cycles();
    	my_enabled_timer = TIMER_ID::interchar_timeout;
    }
    /**
     * @brief Start/enable the response timeout timer.
     *        The response timeout is the maximum waiting time after transmitting a request before assuming the response will not be received
    */
    void enable_response_timeout() {
    	timer_start_time = get_system_cycles();
    	my_enabled_timer = TIMER_ID::repsonse_timeout;
    }
    /**
     * @brief Start/enable the turnaround delay timer.
     *        The turnarnound delay is the minimum time to wait after transmitting a broadcast message before moving to the next transmission
    */
    void enable_turnaround_delay() {
    	timer_start_time = get_system_cycles();
    	my_enabled_timer = TIMER_ID::turnaround_delay;
    }

    /**
     * @brief disables all timers
     */
    void disable_timer () { my_enabled_timer = TIMER_ID::none; }

    /**
     * @brief checks the time and returns the ID of any timer that has expired
     */
    TIMER_ID has_timer_expired () {

    	volatile int64_t timer_start_time_l = timer_start_time;
    	volatile int64_t tnow = get_system_cycles();

    	switch (my_enabled_timer) {
    	case TIMER_ID::repsonse_timeout : if ((int64_t)(tnow - timer_start_time_l) > repsonse_timeout_cycles  ) return TIMER_ID::repsonse_timeout ; break;
    	case TIMER_ID::interchar_timeout: if ((int64_t)(tnow - timer_start_time_l) > interchar_timeout_cycles ) return TIMER_ID::interchar_timeout; break;
    	case TIMER_ID::turnaround_delay : if ((int64_t)(tnow - timer_start_time_l) > turnaround_delay_cycles  ) return TIMER_ID::turnaround_delay ; break;
    	case TIMER_ID::interframe_delay : if ((int64_t)(tnow - timer_start_time_l) > interframe_delay_cycles  ) return TIMER_ID::interframe_delay ; break;
    	case TIMER_ID::none:
    	default:
    		break;
    	}

		return TIMER_ID::none;
    }

    void log_transaction_transmission(Transaction* transaction)
    {
        std::stringstream message;
        message << clock.get_time_microseconds() << "\ttx";
        uint8_t* tx_data = transaction->get_raw_tx_data();
        for (int i = 0; i < transaction->get_tx_buffer_size(); i++)
        {
            message << "\t" << std::setfill('0') << std::setw(2) << std::noshowbase << std::hex << (int)tx_data[i];
        }
        log->write(message.str());
    }

    void log_transaction_response(Transaction* transaction)
    {
        std::stringstream message;
        message << clock.get_time_microseconds() << "\trx";
        uint8_t* rx_data = transaction->get_raw_rx_data();
        for (int i = 0; i < transaction->get_rx_buffer_size(); i++)
        {
            message << "\t" << std::setfill('0') << std::setw(2) << std::noshowbase << std::hex << (int)rx_data[i];
        }

        uint8_t failure_codes = transaction->get_failure_codes();
        if (failure_codes) message << "\t";
        if (failure_codes & (1 << Transaction::RESPONSE_TIMEOUT_ERROR)) message << "Timed out. ";
        if (failure_codes & (1 << Transaction::INTERCHAR_TIMEOUT_ERROR)) message << "Unexpected interchar. ";
        if (failure_codes & (1 << Transaction::UNEXPECTED_RESPONDER)) message << "Wrong address. ";
        if (failure_codes & (1 << Transaction::CRC_ERROR)) message << "Wrong CRC. ";

        log->write(message.str());
    }

    void conclude_transaction(Transaction* transaction)
    {
        enable_interframe_delay();
        if (logging) log_transaction_response(transaction);
        if ((!transaction->is_reception_valid() || transaction->is_error_response()) && transaction->is_important()) {
            if (transaction->get_num_retries() < 5)
            {
                Transaction retry_transaction;
                retry_transaction.generate_retry(transaction);
                messages.insert_next(retry_transaction);
            }
        }
    }

    void flush_remaining_bytes_from_serial_interface()
    {
        while (serial_interface.ready_to_receive())
        {
            serial_interface.receive_byte();
        }
    }
};

}

#endif

