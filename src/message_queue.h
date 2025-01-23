/**
 * @file message_queue.h
 * @author Kali Erickson <kerickson@irisdynamics.com>
 * 
 * @brief  Buffer queue for transmitting and receiving messages through the UART channels.
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


#ifndef MESSAGE_QUEUE_H_
#define MESSAGE_QUEUE_H_

#include "diagnostics_tracker.h"
#include "transaction.h"
#include <list>

namespace orcaSDK
{

/**
 * @class MessageQueue
 * @brief An array implemented queue of MODBUS RTU frame objects for sequential transmittion of commands to remote devices
*/
class MessageQueue {
public:
    /*@brief On construction the message queue and errors are reset.
    */
    MessageQueue(DiagnosticsTracker& diagnostic_counters) :
        diagnostics_tracker(diagnostic_counters)
    {
    	reset();
    }

    /**
     * @brief debugging information
     */
    void printme() {
#ifdef IRISCONTROLS

		PRINTDL("Queue Size: ", size());
		PRINTDL("front: ", front_index);
		PRINTDL("back: ", back_index);
		PRINTDL("active: ", active_index);
#endif 
    	for (Transaction& t : transaction_buffer) {
    		t.printme();
    	}
    }

    /**
     * @brief reset all messages in the queue to be empty
     */
    void reset () {
        transaction_buffer.clear();
    }

    /**
     * @brief Adds the next message requested for transmission to the end of the queue
    */
    void enqueue(Transaction message){
        message.mark_queued();
        transaction_buffer.push_back(message);
    }

    void insert_next(Transaction message)
    {
        message.mark_queued();
        if (transaction_buffer.size() > 0)
        {
            transaction_buffer.insert(++transaction_buffer.begin(), message);
        }
        else
        {
            transaction_buffer.push_front(message);
        }
    }

    /**
     * @brief used to check whether a message is ready to be dequeued
     */
    bool is_response_ready() {
        return transaction_buffer.size() && transaction_buffer.front().is_ready_to_process();
    }


    /**
     * @brief returns a pointer to the message now removed from the queue
     * 
    */
    Transaction dequeue(){
        Transaction ret;
    	if(size()) {
            transaction_buffer.front().mark_dequeued();
            ret = transaction_buffer.front();
            transaction_buffer.pop_front();
    	}
        return ret;
    }

    /**
     * @brief returns a pointer to the active transaction. No checking is done as to the state of the transaction
     */
    Transaction * get_active_transaction () {
    	return &transaction_buffer.front();
    }

    /**
     * @brief returns true when the transaction at the active_index is new and ready to start being sent
     * Advances the active_index when the current message is finished and marks the transaction as sent
     * ie this assumes the caller will transmit the message when this returns true
     * When this returns true, the transaction at active_index is ready to transmit, but hasn't been started yet
     */
    bool available_to_send() {

        if (size() == 0) return false;

        return transaction_buffer.front().is_queued();
    }

    void mark_active_message_sent()
    {
        transaction_buffer.front().mark_sent();
    }

	/**
	 * @brief Determine the number of messages currently in the queue
	 * @return The number of messages in the queue
	*/
   size_t size(){
	   return transaction_buffer.size();
   }

private:
    DiagnosticsTracker& diagnostics_tracker;
    std::list<Transaction> transaction_buffer;  
};

}

#endif

