/*
* Copyright (C) 2013, Osnabrück University
* Copyright (C) 2017, Ing.-Buero Dr. Michael Lehning, Hildesheim
* Copyright (C) 2017, SICK AG, Waldkirch
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Osnabrück University nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission.
*     * Neither the name of SICK AG nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission
*     * Neither the name of Ing.-Buero Dr. Michael Lehning nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
*  Last modified: 12th Dec 2017
*
*      Authors:
*              Michael Lehning <michael.lehning@lehning.de>
*         Jochen Sprickerhof <jochen@sprickerhof.de>
*         Martin Günther <mguenthe@uos.de>
*
* Based on the TiM communication example by SICK AG.
*
*/

#ifdef _MSC_VER
#pragma warning(disable: 4996)
#pragma warning(disable: 4267)
#pragma warning(disable: 4101)   // C4101: "e" : Unreferenzierte lokale Variable
#define _WIN32_WINNT 0x0501

#endif

#include <sick_scan/sick_scan_common_tcp.h>
#include <boost/asio.hpp>
#include <boost/lambda/lambda.hpp>
#include <algorithm>
#include <iterator>
#include <boost/lexical_cast.hpp>

#ifdef _MSC_VER
#include "sick_scan/rosconsole_simu.hpp"
#endif

static int getDiagnosticErrorCode()
{
#ifdef _MSC_VER
#undef ERROR
	return(2);
#else
	return(diagnostic_msgs::DiagnosticStatus::ERROR);
#endif
}
namespace sick_scan
{

	SickScanCommonTcp::SickScanCommonTcp(const std::string &hostname, const std::string &port, int &timelimit, SickGenericParser* parser)
		:
		SickScanCommon(parser),
		socket_(io_service_),
		deadline_(io_service_),
		hostname_(hostname),
		port_(port),
		timelimit_(timelimit)
	{
		// Set up the deadline actor to implement timeouts.
		// Based on blocking TCP example on:
		// http://www.boost.org/doc/libs/1_46_0/doc/html/boost_asio/example/timeouts/blocking_tcp_client.cpp
		deadline_.expires_at(boost::posix_time::pos_infin);
		checkDeadline();
	}

	SickScanCommonTcp::~SickScanCommonTcp()
	{
		stop_scanner();
		close_device();
	}

	using boost::asio::ip::tcp;
	using boost::lambda::var;
	using boost::lambda::_1;

	int SickScanCommonTcp::init_device()
	{
		// Resolve the supplied hostname
		tcp::resolver::iterator iterator;
		try
		{
			tcp::resolver resolver(io_service_);
			tcp::resolver::query query(hostname_, port_);
			iterator = resolver.resolve(query);
		}
		catch (boost::system::system_error &e)
		{
			ROS_FATAL("Could not resolve host: ... (%d)(%s)", e.code().value(), e.code().message().c_str());
			diagnostics_.broadcast(getDiagnosticErrorCode(), "Could not resolve host.");
			return ExitError;
		}

		// Try to connect to all possible endpoints
		boost::system::error_code ec;
		bool success = false;
		for (; iterator != tcp::resolver::iterator(); ++iterator)
		{
			std::string repr = boost::lexical_cast<std::string>(iterator->endpoint());
			socket_.close();

			// Set the time out length
			ROS_INFO("Waiting %i seconds for device to connect.", timelimit_);
			deadline_.expires_from_now(boost::posix_time::seconds(timelimit_));

			ec = boost::asio::error::would_block;
			ROS_DEBUG("Attempting to connect to %s", repr.c_str());
			socket_.async_connect(iterator->endpoint(), boost::lambda::var(ec) = _1);

			// Wait until timeout
			do io_service_.run_one(); while (ec == boost::asio::error::would_block);

			if (!ec && socket_.is_open())
			{
				success = true;
				ROS_INFO("Succesfully connected to %s", repr.c_str());
				break;
			}
			ROS_ERROR("Failed to connect to %s", repr.c_str());
		}

		// Check if connecting succeeded
		if (!success)
		{
			ROS_FATAL("Could not connect to host %s:%s", hostname_.c_str(), port_.c_str());
			diagnostics_.broadcast(getDiagnosticErrorCode(), "Could not connect to host.");
			return ExitError;
		}

		input_buffer_.consume(input_buffer_.size());

		return ExitSuccess;
	}

	int SickScanCommonTcp::close_device()
	{
		if (socket_.is_open())
		{
			try
			{
				socket_.close();
			}
			catch (boost::system::system_error &e)
			{
				ROS_ERROR("An error occured during closing of the connection: %d:%s", e.code().value(), e.code().message().c_str());
			}
		}
		return 0;
	}

	void SickScanCommonTcp::checkDeadline()
	{
		if (deadline_.expires_at() <= boost::asio::deadline_timer::traits_type::now())
		{
			// The reason the function is called is that the deadline expired. Close
			// the socket to return all IO operations and reset the deadline
			socket_.close();
			deadline_.expires_at(boost::posix_time::pos_infin);
		}

		// Nothing bad happened, go back to sleep
		deadline_.async_wait(boost::bind(&SickScanCommonTcp::checkDeadline, this));
	}

	int SickScanCommonTcp::readWithTimeout(size_t timeout_ms, char *buffer, int buffer_size, int *bytes_read, bool *exception_occured)
	{
		// Set up the deadline to the proper timeout, error and delimiters
		deadline_.expires_from_now(boost::posix_time::milliseconds(timeout_ms));
		const char end_delim = static_cast<char>(0x03);
		ec_ = boost::asio::error::would_block;
		bytes_transfered_ = 0;

		// Read until 0x03 ending indicator
		boost::asio::async_read_until(
			socket_,
			input_buffer_,
			end_delim,
			boost::bind(
				&SickScanCommonTcp::handleRead,
				this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred
			)
		);
		do io_service_.run_one(); while (ec_ == boost::asio::error::would_block);

		if (ec_)
		{
			// would_block means the connectio is ok, but nothing came in in time.
			// If any other error code is set, this means something bad happened.
			if (ec_ != boost::asio::error::would_block)
			{
				ROS_ERROR("sendSOPASCommand: failed attempt to read from socket: %d: %s", ec_.value(), ec_.message().c_str());
				diagnostics_.broadcast(getDiagnosticErrorCode(), "sendSOPASCommand: exception during read_until().");
				if (exception_occured != 0)
					*exception_occured = true;
			}

			// For would_block, just return and indicate nothing bad happend
			return ExitError;
		}

		// Avoid a buffer overflow by limiting the data we read
		size_t to_read = bytes_transfered_ > buffer_size - 1 ? buffer_size - 1 : bytes_transfered_;
		size_t i = 0;
		std::istream istr(&input_buffer_);
		if (buffer != 0)
		{
			istr.read(buffer, to_read);
			buffer[to_read] = 0;

			// Consume the rest of the message if necessary
			if (to_read < bytes_transfered_)
			{
				ROS_WARN("Dropping %zu bytes to avoid buffer overflow", bytes_transfered_ - to_read);
				input_buffer_.consume(bytes_transfered_ - to_read);
			}
		}
		else
			// No buffer was provided, just drop the data
			input_buffer_.consume(bytes_transfered_);

		// Set the return variable to the size of the read message
		if (bytes_read != 0)
			*bytes_read = to_read;

		return ExitSuccess;
	}

	/**
	 * Send a SOPAS command to the device and print out the response to the console.
	 */
	int SickScanCommonTcp::sendSOPASCommand(const char* request, std::vector<unsigned char> * reply)
	{
		if (!socket_.is_open()) {
			ROS_ERROR("sendSOPASCommand: socket not open");
			diagnostics_.broadcast(getDiagnosticErrorCode(), "sendSOPASCommand: socket not open.");
			return ExitError;
		}

		int sLen = 0;
		int preambelCnt = 0;
		bool cmdIsAscii = true;

		if (request != NULL)
		{
			sLen = strlen(request);
			preambelCnt = 0; // count 0x02 bytes to decide between ascii and binary command
			if (sLen >= 4)
			{
				for (int i = 0; i < 4; i++) {
					if (request[i] == 0x02)
					{
						preambelCnt++;
					}
				}
			}

			if (preambelCnt < 4) {
				cmdIsAscii = true;
			}
			else
			{
				cmdIsAscii = false;
			}
			int msgLen = 0;
			if (cmdIsAscii == true) {
				msgLen = strlen(request);
			}
			else
			{
				int dataLen = 0;
				for (int i = 4; i < 8; i++)
				{
					dataLen |= ((unsigned char)request[i] << (7 - i) * 8);
				}
				msgLen = 8 + dataLen + 1; // 8 Msg. Header + Packet +
			}
			/*
			 * Write a SOPAS variable read request to the device.
			 */
			try
			{
				boost::asio::write(socket_, boost::asio::buffer(request, msgLen));
			}
			catch (boost::system::system_error &e)
			{
				ROS_ERROR("write error for command: %s", request);
				diagnostics_.broadcast(getDiagnosticErrorCode(), "Write error for sendSOPASCommand.");
				return ExitError;
			}
		}

		// Set timeout in 5 seconds
		const int BUF_SIZE = 1000;
		char buffer[BUF_SIZE];
		int bytes_read;
		if (readWithTimeout(1000, buffer, BUF_SIZE, &bytes_read, 0) == ExitError)
		{
			ROS_ERROR_THROTTLE(1.0, "sendSOPASCommand: no full reply available for read after 1s");
			diagnostics_.broadcast(getDiagnosticErrorCode(), "sendSOPASCommand: no full reply available for read after 5 s.");
			return ExitError;
		}

		if (reply)
		{
			reply->resize(bytes_read);
			std::copy(buffer, buffer + bytes_read, &(*reply)[0]);
		}

		return ExitSuccess;
	}

	int SickScanCommonTcp::get_datagram(unsigned char* receiveBuffer, int bufferSize, int* actual_length)
	{
		if (!socket_.is_open()) {
			ROS_ERROR("get_datagram: socket not open");
			diagnostics_.broadcast(getDiagnosticErrorCode(), "get_datagram: socket not open.");
			return ExitError;
		}

		/*
		 * Write a SOPAS variable read request to the device.
		 */
		std::vector<unsigned char> reply;

		// Wait at most 1000ms for a new scan
		size_t timeout = 1000;
		bool exception_occured = false;

		char *buffer = reinterpret_cast<char *>(receiveBuffer);

		if (readWithTimeout(timeout, buffer, bufferSize, actual_length, &exception_occured) != ExitSuccess)
		{
			ROS_ERROR_THROTTLE(1.0, "get_datagram: no data available for read after %zu ms", timeout);
			diagnostics_.broadcast(getDiagnosticErrorCode(), "get_datagram: no data available for read after timeout.");

			// Attempt to reconnect when the connection was terminated
			if (!socket_.is_open())
			{
#ifdef _MSC_VER
				Sleep(1000);
#else
				sleep(1);
#endif
				ROS_INFO("Failure - attempting to reconnect");
				return init();
			}

			return exception_occured ? ExitError : ExitSuccess;    // keep on trying
		}

		return ExitSuccess;
	}

} /* namespace sick_scan */
