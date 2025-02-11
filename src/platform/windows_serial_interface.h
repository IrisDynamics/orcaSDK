/**
 * @file windows_modbus_client.h
 * @author Kate Colwell <kcolwell@irisdynamics.com>
 *
 * @brief  Virtual device driver for Modbus client serial communication using the Windows API
 *
 * This class extends the virtual ModbusClient base class
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

#pragma once 

#include "../serial_interface.h"
#include<iostream>
#include <windows.h>
#include <timeapi.h>
#include <vector>
#include <sstream>
#include "../modbus_client.h"
#include "../transaction.h"
#include <mutex>
#include <deque>

namespace orcaSDK
{

 /**
  * @class windows_SerialInterface
  * @brief Extension of the ModbusClient virtual class that implements functions for the Eagle K20's timers, and interrupts. UART channel specific functions to be
 */
static void reset_overlapped_struct(OVERLAPPED& o);

class windows_SerialInterface : public SerialInterface {
public:
    windows_SerialInterface() = default;

    /**
    * @brief ends the listening thread, purges the comport and closes it
    */
    ~windows_SerialInterface() {
        clean_up_handles();
    }

    void close_serial_port() override {
        clean_up_handles();
    }

    /**
     * @brief Intializes the com port settings
     * @param baud The baud rate as defined in the client_config.h file
    */
    OrcaError open_serial_port(int serial_port_number, unsigned int baud) override {
        initialize_and_acquire_resources(serial_port_number, baud);
        if (!serial_success) return OrcaError(true, "Could not obtain serial port.");
        return { false, "" };
    }

    bool ready_to_send() override {
        return serial_success && !write_in_progress;
    }

    /**
     * @brief If there are bytes left to send, add them to a buffer, then send them all together.
    */
    //need messages to be switched to protected, not private 
    void tx_enable(size_t) override {
        if (!serial_success) return; //Don't write if we don't have a device to connect to
        if (write_in_progress) return; //Don't write if the previous write hasn't completed

        std::lock_guard<std::mutex> light_lock{light_mutex};
        std::lock_guard<std::mutex> backend_lock{backend_thread_mutex};

        activeBuffer = sendBuffer;

        initiate_write(activeBuffer);

        sendBuffer.clear();
    }

    /**
     * @brief Loads the send buffer with the next byte
     * @param byte		The byte to be transmitted.
     */
    void send_byte(uint8_t data) override {
        std::lock_guard<std::mutex> light_lock{ light_mutex };
        std::lock_guard<std::mutex> backend_lock{ backend_thread_mutex };

        sendBuffer.push_back(data);
    }

    /**
     * @brief Adjust the baud rate
     * @param baud_rate the new baud rate in bps
     * this method overrides the modbus default delay
    */
    void adjust_baud_rate(uint32_t baud_rate_bps) override {
        std::lock_guard<std::mutex> light_lock{ light_mutex };
        std::lock_guard<std::mutex> backend_lock{ backend_thread_mutex };

        GetCommState(hSerial, &dcbSerialParams);

        dcbSerialParams.BaudRate = baud_rate_bps;

        SetCommState(hSerial, &dcbSerialParams);
    }

    uint8_t receive_byte() override {
        std::lock_guard<std::mutex> light_lock{ light_mutex };
        std::lock_guard<std::mutex> backend_lock{ backend_thread_mutex };

        char out_byte = rcvBuf.front();
        rcvBuf.pop_front();

        return out_byte;
    }

    bool ready_to_receive() override {
        std::lock_guard<std::mutex> light_lock{ light_mutex };
        std::lock_guard<std::mutex> backend_lock{ backend_thread_mutex };

        return !rcvBuf.empty();
    }

private:
    static constexpr size_t maxBufferSize = 512;
    std::mutex light_mutex;
    std::mutex backend_thread_mutex;
    std::thread backend_thread;
    std::atomic<bool> run_backend_thread;

#pragma region WRITE_HANDLING
    //Interface between async backend and frontend
    std::deque<char> sendBuffer;
    std::deque<char> activeBuffer;
    std::atomic<bool> write_in_progress = false;
    OVERLAPPED write_overlapped{}; //Synchronization object for asynchronous writes
    bool write_event_handle_created = false;
    std::array<char, maxBufferSize> writeCStyleBuffer{}; //Just for WriteFile interaction

    void initiate_write(const std::deque<char>& buffer)
    {
        DWORD numItemsInBuffer = static_cast<DWORD>(buffer.size());
        for (size_t i = 0; i < numItemsInBuffer; i++) {
            writeCStyleBuffer[i] = buffer[i];
        }

        reset_overlapped_struct(write_overlapped);
        if (!WriteFile(hSerial, writeCStyleBuffer.data(), numItemsInBuffer, NULL, &write_overlapped)) {
            DWORD error_code = GetLastError();

            if (error_code == ERROR_IO_PENDING)
            {
                //ERROR_IO_PENDING indicates a successfully queued asynchronous write
                write_in_progress = true;
            }
            else {
                std::wstring error_str = L"Error initiating a write. Error code: " + std::to_wstring(GetLastError()) + L"\n";
                OutputDebugStringW(error_str.c_str());
            }
        }
        else
        {
            //Returned true, write completed before WriteFile returned
            handle_write_complete();
        }
    }

    void handle_write_complete()
    {
        DWORD num_bytes_written;
        BOOL result = GetOverlappedResult(hSerial, &write_overlapped, &num_bytes_written, TRUE);
        if (!result)
        {
            DWORD error_code = GetLastError();
            if (error_code != ERROR_OPERATION_ABORTED) //Aborting may occur as part of disconnecting
            {
                std::wstring error_str = L"Error writing bytes. Error code: " + std::to_wstring(GetLastError()) + L"\n";
                OutputDebugStringW(error_str.c_str());
            }
        }

        activeBuffer.clear();
        write_in_progress = false;
    }
#pragma endregion
#pragma region READ_HANDLING
    std::deque<char> rcvBuf;
    OVERLAPPED read_overlapped{}; //Synchronization object for asynchronous reads 
    bool read_event_handle_created = false;
    bool read_in_progress = false;
    std::array<char, maxBufferSize> readCStyleBuffer{}; //Just for ReadFile interaction

    void initiate_read()
    {
        reset_overlapped_struct(read_overlapped);
        if (!ReadFile(hSerial, readCStyleBuffer.data(), 1, NULL, &read_overlapped)) {
            DWORD error_code = GetLastError();

            switch (error_code)
            {
            case ERROR_IO_PENDING:
                //ERROR_IO_PENDING indicates a successfully queued asynchronous operation
                read_in_progress = true;
                break;
            default:
                std::wstring error_str = L"Error initiating read. Error code: " + std::to_wstring(GetLastError()) + L"\n";
                OutputDebugStringW(error_str.c_str());
                break;
            }
        }
        else {
            //Returned true, read completed before ReadFile returned
            handle_read_complete();
        }
    }

    void handle_read_complete()
    {
        read_in_progress = false;

        DWORD num_bytes_read;
        BOOL result = GetOverlappedResult(hSerial, &read_overlapped, &num_bytes_read, TRUE);
        if (!result)
        {
            DWORD error_code = GetLastError();
            switch (error_code)
            {
            case ERROR_OPERATION_ABORTED:
                break;
            default:
                std::wstring error_str = L"Error writing bytes. Error code: " + std::to_wstring(GetLastError()) + L"\n";
                OutputDebugStringW(error_str.c_str());
                break;
            }
        }
        for (DWORD i = 0; i < num_bytes_read; i++)
        {
            rcvBuf.push_back(readCStyleBuffer[i]);
        }
        
        if (run_backend_thread) initiate_read();
    }
#pragma endregion

    //IO file handle stuff
    HANDLE hSerial{ nullptr }; //The com port file handle 
    DCB dcbSerialParams = { 0 }; //Serial parameters for file handle
    bool serial_success = false;     //flag bool to indicate if the handle to the serial port is obtained
    COMMTIMEOUTS timeouts{
            1,
            0,
            0,
            0,
            0 
    };

    void initialize_and_acquire_resources(int channel_number, int baud)
    {
        if (serial_success)
        {
            LPCWSTR getErr = L"Calling init, but comport already acquired\n";
            OutputDebugStringW(getErr);
            return;
        } 
        else if(!acquire_port(channel_number)) {
            return;
        }
        else {
            serial_success = true;
        }

        //set comport parameters - this is where init used to be called. 
        //get current state
        dcbSerialParams.DCBlength = sizeof(DCB);
        if (!GetCommState(hSerial, &dcbSerialParams)) {
            LPCWSTR getErr = L"Error getting current port state\n";
            OutputDebugStringW(getErr);
        }
        //set new state
        dcbSerialParams.BaudRate = baud;
        dcbSerialParams.ByteSize = 8;
        dcbSerialParams.StopBits = ONESTOPBIT;
        dcbSerialParams.Parity = EVENPARITY;
        // dcbSerialParams.fRtsControl = RTS_CONTROL_ENABLE;
        if (!SetCommState(hSerial, &dcbSerialParams)) {
            LPCWSTR paramErr = L"Error setting serial port state\n";
            OutputDebugStringW(paramErr);
        }

        //set comm Mask - will notify when a byte arrives in the port 
        if (!SetCommMask(hSerial, EV_RXCHAR)) {
            LPCWSTR maskErr = L"Error setting port com mask\n";
            OutputDebugStringW(maskErr);
        }

        //set comm Mask - will notify when a byte arrives in the port 
        if (!SetCommTimeouts(hSerial, &timeouts)) {
            LPCWSTR maskErr = L"Error setting com timeouts\n";
            OutputDebugStringW(maskErr);
        }

        //Create event handles for OVERLAPPED structs
        write_overlapped.hEvent = CreateEvent(NULL, TRUE, FALSE, "OrcaSDKWriteEvent");
        if (write_overlapped.hEvent == NULL) {
            LPCWSTR eventErr = L"Error setting write overlapped event\n";
            OutputDebugStringW(eventErr);
        }
        else
        {
            write_event_handle_created = true;
        }
        read_overlapped.hEvent = CreateEvent(NULL, TRUE, FALSE, "OrcaSDKReadEvent");
        if (read_overlapped.hEvent == NULL) {
            LPCWSTR eventErr = L"Error setting write overlapped event\n";
            OutputDebugStringW(eventErr);
        }
        else
        {
            read_event_handle_created = true;
        }

        //set everything to a clear state 
        activeBuffer.clear();
        sendBuffer.clear();
        rcvBuf.clear();

        //Set the backend thread running
        start_new_backend_thread();
        initiate_read();
    }

    bool acquire_port(int port_num) {
        //get port number from channel parameters
        std::ostringstream portOsStr;
        portOsStr << "\\\\.\\COM" << port_num;
        std::string portStr(portOsStr.str());
        LPCSTR portName = portStr.c_str();

        //open handle to comport
        HANDLE hSerialCheck = CreateFileA(portName, GENERIC_READ | GENERIC_WRITE | OPEN_ALWAYS, 0, 0, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, 0);  //FILE_FLAG_OVERLAPPED //FILE_ATTRIBUTE_NORMAL
        if (hSerialCheck == INVALID_HANDLE_VALUE) {
            std::wstring portErr = L"Error opening comport. Error number: " + std::to_wstring(GetLastError()) + L"\n";
            OutputDebugStringW(portErr.c_str());
            serial_success = false;
        }
        else {
            hSerial = hSerialCheck;
            serial_success = true;
        }

        return serial_success;
    }

    void backend_thread_loop()
    {
        while (run_backend_thread)
        {
            std::unique_lock<std::mutex> thread_lock{ backend_thread_mutex };
            if (write_in_progress && HasOverlappedIoCompleted(&write_overlapped))
            {
                handle_write_complete();
            }
            if (read_in_progress && HasOverlappedIoCompleted(&read_overlapped))
            {
                handle_read_complete();
            }

            thread_lock.unlock();

            std::this_thread::yield();

            //Locking this mutex right before leaving scope makes it easy 
            // for another thread to interrupt the hot loop
            std::lock_guard<std::mutex> light_lock{ light_mutex };
        }
    }

    //Do not lock the backend before calling this method! It requires the backend to complete
    // its current iteration before this method will complete
    void clean_up_handles()
    {
        run_backend_thread = false;

        if (backend_thread.joinable()) backend_thread.join();

        WaitForSingleObject(write_overlapped.hEvent, 50);
        WaitForSingleObject(read_overlapped.hEvent, 50);

        if (read_event_handle_created) CloseHandle(read_overlapped.hEvent);
        if (write_event_handle_created) CloseHandle(write_overlapped.hEvent);
        if (serial_success) CloseHandle(hSerial);

        serial_success = false;
    }    

    void start_new_backend_thread()
    {
        run_backend_thread = true;
        backend_thread = std::thread{&windows_SerialInterface::backend_thread_loop, this};
    }
};

static void reset_overlapped_struct(OVERLAPPED& o)
{
    o.Internal = 0;
    o.InternalHigh = 0;
    o.Offset = 0;
    o.OffsetHigh = 0;
    o.Pointer = 0;

    bool reset_successful = ResetEvent(o.hEvent);
    if (!reset_successful) {
        LPCWSTR eventErr = L"Error resetting overlapped event\n";
        OutputDebugStringW(eventErr);
    }
}

extern windows_SerialInterface modbus_client;

}