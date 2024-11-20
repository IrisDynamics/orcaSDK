#pragma once

#include "tools/log_interface.h"
#include <iostream>
#include <fstream>
#include <ctime>
#include <iomanip>
#include <chrono>

class Log : public LogInterface {
public:
	enum class TimestampType {
		CurrentDateTime,
		DurationSinceOpening
	};

	Log(TimestampType _timestamp_type = TimestampType::CurrentDateTime) :
		timestamp_type_setting(_timestamp_type)
	{}
	~Log() {
		close();
	}
	Log(Log& other) = delete;
	Log& operator=(Log& other) = delete;

	/**
	 *	@brief	Returns true if the log has a currently opened file
	 */
	int is_open() { return file.is_open(); }

	/**
	 *	@brief	Writes the contents of the str parameter to the file. Also writes a timestamp if 
	 *			verbose mode is on.
	 *	@param	std::string str  The log message to be written
	 *	@throws	std::runtime_error	If this method is called on a log without an opened file
	 */
	OrcaError write(const std::string& str) override {
		return write_internal(str, timestamp_type_setting);
	}

	/**
	 *	@brief	Opens a file located at the given path, creating one if it doesn't exist, and appending 
	 *			to the file if it does exist.
	 *	@param	std::string path  The file location of the desired log file. Begins relative paths at 
								  the calling .exe
	 *	@param	std::string file_ext  The file extension of the desired file, defaults to .txt
	 *	@throws	std::runtime_error	If this method fails to open the file, or if a file is already open
	 *	@notes	If verbose mode is on, writes a message signaling a successful open
	 */
	OrcaError open(const std::string& path) override {
		std::string full_name = path;

		if (is_open()) return OrcaError(true, "Could not open file: " + full_name + ". The file: " + file_name + " is already open.");

		file_name = full_name;

		file.open(full_name, std::ios::out | std::ios::app | std::ios::binary);
		if (!is_open()) return OrcaError(true, "Failed to open log file: " + path);

		start_time = std::chrono::high_resolution_clock::now();

		if (verbose_mode) write_internal("Opened File", TimestampType::CurrentDateTime);

		return OrcaError(false, "");
	}

	/**
	 *	@brief	Closes the current file, if it is open
	 *	@notes	If verbose mode is on, writes a message signaling closing the file
	 */
	void close() {
		if (is_open() && verbose_mode) write_internal("Closed File", TimestampType::CurrentDateTime);
		file_name = "";
		file.close();
	}

	/**
	 *	@brief	Turns verbose mode on or off. If verbose mode is on, then, in addition to user defined
	 *			writes, the log will:
	 *				1. Include a timestamp with every log write
	 *				2. Write a message upon opening the file
	 *				3. Write a message upon closing the file
	 */
	void set_verbose_mode(bool active) {
		verbose_mode = active;
	}

	// Included for interoperability between log implementations
	void add(const char* name) {
		open(name);
	}

private:
	bool verbose_mode = true;

	std::string file_name = "";

	std::ofstream file;

	std::chrono::high_resolution_clock::time_point start_time;

	TimestampType timestamp_type_setting = TimestampType::CurrentDateTime;

	std::string as_seconds(std::chrono::duration<float> duration) {
		return std::to_string(duration.count()) + "s";
	}

	OrcaError write_internal(std::string str, TimestampType timestamp_type)
	{
		if (!file.is_open()) return OrcaError(true, "Tried to write to unopened log file " + file_name + ".");
		if (verbose_mode) str = get_timestamp(timestamp_type) + str;
		file << str << "\r\n" << std::flush;
		return OrcaError(false, "");
	}

	// Referenced from awesoon's answer here: https://stackoverflow.com/questions/16357999/current-date-and-time-as-string
	std::string get_timestamp(TimestampType timestamp_type) {
		if (timestamp_type == TimestampType::CurrentDateTime) {
			std::time_t now = std::time(0);

			std::tm localtm;
			localtime_s(&localtm, &now);

			std::stringstream ss;
			try {
				ss << std::put_time(&localtm, "%a %d %b %Y %H:%M:%S: ");
			}
			catch (std::exception& e)
			{
				std::cout << e.what() << std::endl;
			}

			return ss.str();
		}
		else if (timestamp_type == TimestampType::DurationSinceOpening) {
			auto curr_time = std::chrono::high_resolution_clock::now();
			std::chrono::duration<float> time_since_start = curr_time - start_time;
			return as_seconds(time_since_start) + ": ";
		}
		else {
			return "Unknown Timestamp: ";
		}
	}
};
