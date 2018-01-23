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
#include <sick_scan/tcp/colaa.hpp>
#include <sick_scan/tcp/colab.hpp>

#include <boost/asio.hpp>
#include <boost/lambda/lambda.hpp>
#include <algorithm>
#include <iterator>
#include <boost/lexical_cast.hpp>
#include <vector>
#ifdef _MSC_VER
#include "sick_scan/rosconsole_simu.hpp"
#endif

std::vector<unsigned char> exampleData(65536);
std::vector<unsigned char> receivedData(65536);
static long receivedDataLen = 0;
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
    this->setReplyMode(0);
		// io_service_.setReadCallbackFunction(boost::bind(&SopasDevice::readCallbackFunction, this, _1, _2));

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


    void SickScanCommonTcp::disconnectFunction()
    {

    }

    void SickScanCommonTcp::disconnectFunctionS(void *obj)
    {
      if (obj != NULL)
      {
        ((SickScanCommonTcp *)(obj))->disconnectFunction();
      }
    }

		void SickScanCommonTcp::readCallbackFunctionS(void* obj, UINT8* buffer, UINT32& numOfBytes)
		{
			((SickScanCommonTcp*)obj)->readCallbackFunction(buffer, numOfBytes);
		}


    void SickScanCommonTcp::setReplyMode(int _mode)
    {
      m_replyMode = _mode;
    }
    int SickScanCommonTcp::getReplyMode()
    {
      return(m_replyMode);
    }

    /**
 * Read callback. Diese Funktion wird aufgerufen, sobald Daten auf der Schnittstelle
 * hereingekommen sind.
 */
		void SickScanCommonTcp::readCallbackFunction(UINT8* buffer, UINT32& numOfBytes)
		{
      // should be member variable in the future


#if 0

      this->recvQueue.push(std::vector<unsigned char>(buffer, buffer + numOfBytes));

      if (this->getReplyMode() == 0)
      {
        this->recvQueue.push(std::vector<unsigned char>(buffer, buffer + numOfBytes));
        return;
      }
#endif
        if (numOfBytes < 9)
        {
          return;
        }

        // check magic word for cola B
        if ((buffer[0] == 0x02 && buffer[1] == 0x02 && buffer[2] == 0x02 && buffer[3] == 0x02) || m_alreadyReceivedBytes > 0)
        {
          std::string command;
          UINT16 nextData = 4;
          UINT32 numberBytes = numOfBytes;
          if (m_alreadyReceivedBytes == 0)
          {
            const char *packetKeyWord = "sSN LMDscandata";
            m_lastPacketSize = colab::getIntegerFromBuffer<UINT32>(buffer, nextData);
						//
						m_lastPacketSize += 9; // Magic number + CRC

						// Check for "normal" command reply
            if (strncmp((char *)(buffer + 8),packetKeyWord, strlen(packetKeyWord)) != 0)
            {
							// normal command reply
              this->recvQueue.push(std::vector<unsigned char>(buffer, buffer + numOfBytes));
              return;
            }

            // probably a scan
            if (m_lastPacketSize > 4000)
            {
              INT16 topmostLayerAngle = 1350 * 2 - 62; // for identification of first layer of a scan
              UINT16 layerPos = 24 + 26;
              INT16 layerAngle = colab::getIntegerFromBuffer<INT16>(buffer, layerPos);

              if (layerAngle == topmostLayerAngle)
              {
                // wait for all 24 layers
                // m_lastPacketSize = m_lastPacketSize * 24;

                // traceDebug(MRS6xxxB_VERSION) << "Received new scan" << std::endl;
              }
            }
          }

          // copy
          memcpy(m_packetBuffer + m_alreadyReceivedBytes, buffer, numOfBytes);
          m_alreadyReceivedBytes += numberBytes;

          if (m_alreadyReceivedBytes < m_lastPacketSize)
          {
            // wait for completeness of packet
            return;
          }

          m_alreadyReceivedBytes = 0;
          recvQueue.push(std::vector<unsigned char>(m_packetBuffer, m_packetBuffer + m_lastPacketSize));
#if 0
          command = colab::getCommandStringFromBuffer(m_packetBuffer);
          UINT32 packetSize = colab::getIntegerFromBuffer<UINT32>(m_packetBuffer, nextData);
          std::string identifier = colab::getIdentifierFromBuffer(m_packetBuffer, nextData, m_lastPacketSize);

          // scan event
          if ((command.compare("SN") == 0) && (identifier.compare("LMDscandata") == 0))
          {
            // if we have collected all 24 scans from all layers
            if (m_lastPacketSize > packetSize) // more data received then necessary for one scan packet
            {
              // iterate through layer
              for (UINT32 index = 0; index < 24; ++index)
              {
                UINT32 startPos = index * (packetSize + 2 * sizeof(UINT32) + sizeof(UINT8)); // magic word + packetsize + crc
                unsigned char *startPtr = &m_packetBuffer[startPos + nextData];
                unsigned char *endPtr = startPtr + packetSize - nextData;

                recvQueue.push(std::vector<unsigned char>(startPtr, endPtr));
                // readAndDecodeMRS6xxxScan(&m_packetBuffer[startPos + nextData], packetSize - nextData);
              }
            }
            else
            {
              unsigned char *startPtr = &m_packetBuffer[nextData];
              unsigned char *endPtr = startPtr + packetSize - nextData;
              //  readAndDecodeMRS6xxxScan(&m_packetBuffer[nextData], packetSize - nextData);
              recvQueue.push(std::vector<unsigned char>(startPtr, endPtr));
            }
          }
            // answer to register scan event
          else if ((command.compare(0, 2, "EA") == 0) && (identifier.compare(0, 11, "LMDscandata") == 0))
          {
            if (m_packetBuffer[nextData] == 1)
            {
              // was successful
            }
          }
#endif
        }


		}


		int SickScanCommonTcp::init_device()
    {
      int portInt;
      sscanf(port_.c_str(),"%d", &portInt);
      m_nw.init(hostname_, portInt, disconnectFunctionS, (void*)this);
			m_nw.setReadCallbackFunction(readCallbackFunctionS,(void*)this);
      m_nw.connect();
      return ExitSuccess;
    }

    #if 0
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

			// tryBroadCastForMoreInfo();
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
#endif

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

	void SickScanCommonTcp::handleRead(boost::system::error_code error, size_t bytes_transfered)
	{
		ec_ = error;
		bytes_transfered_ += bytes_transfered;
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
#if 0
	std::size_t my_completion_handler(
		// Result of latest async_read_some operation.
		const boost::system::error_code& error,
		// Number of bytes transferred so far.
		std::size_t bytes_transferred
	)
	{
		static int state = 0;
		static unsigned long dataLen = 0;
		static int bytes_total_transferred = 0;
		bool validData = true;
		std::size_t numToRead = 0;

		if (bytes_transferred == 0)
		{
			bytes_total_transferred = 0;
			receivedData.clear();
			receivedData.reserve(65536);
		}
		else
		{
			bytes_total_transferred += bytes_transferred;
		}
		int receivedBytesNum = receivedData.size();
		if (bytes_transferred > 1) // payload
		{
			numToRead = 0;
			state = 0;
		}
		else
		{
			if (receivedBytesNum < 8)
			{
				if (bytes_transferred != 0)
				{
					receivedData.push_back(exampleData[receivedBytesNum]);
				}
				if (receivedData.size() == 8)
				{

					int cnt0x02 = 0;

					for (int i = 0; i < 4; i++)
					{
						if (receivedData[i] == 0x02)
						{
							cnt0x02++;
						}
					}
					if (cnt0x02 < 4)
					{
						validData = false;
						if (receivedData[0] == 0x02)
						{
							printf("ASCII-Format for SOPAS??? Please check - Binary format required\n");
						}
						else
						{
							printf("INVALID DATA FORMAT! Cancelling data transfer\n");
						}
					}
					if (validData)
					{
						for (int i = 4; i < 8; i++)
						{
							int relOffset = i - 4;
							dataLen |= receivedData[i] << (8 * (7 - i));
						}
						dataLen += 1; // wg. CRC
						state = 1;
					}

					// interpret data
					printf("Test\n");
				}
			}
			printf("Bytes transferred: %d\n", bytes_transferred);
			if (validData)
			{

				if (dataLen != 0)
				{
					numToRead = dataLen;
				}
				else
				{
					numToRead = bytes_total_transferred + 1;
				}
			}
			else
			{
				numToRead = 0;
				state = 0;
			}
		}
		printf("NumToRead: %d\n", numToRead);
		return(numToRead);
	}
#endif

#if 1
	std::size_t my_completion_handler(
		// Result of latest async_read_some operation.
		const boost::system::error_code& error,
		// Number of bytes transferred so far.
		std::size_t bytes_transferred
	)
	{

		static int state = 0;
		static int numToRead = 0;
		static int payloadSize = 0;
		if (bytes_transferred == 0)  // initial
		{
			receivedDataLen = 0;
			numToRead = 8;
			state = 1;
		}
		else
		{
			receivedDataLen = bytes_transferred;
			switch (state)
			{
			case 1: // read header
			{
				numToRead = 0;
				int cnt0x02 = 0;
				for (int i = 0; i < 4; i++)
				{
					if (exampleData[i] == 0x02)
					{
						cnt0x02++;
					}
				}
				if (cnt0x02 == 4)
				{
					for (int i = 4; i < 8; i++)
					{
						int relOffset = i - 4;
						numToRead |= exampleData[i] << (8 * (7 - i));
					}
					numToRead += 1;
					payloadSize = numToRead;
					state = 2;
				}
				else
				{
					// out of sync - what shell we do now?
					numToRead = 0;  // invalid packet - cancle reading
				}
			}
			break;
			case 2: // read payload
				numToRead = (receivedDataLen - 8 - payloadSize);
				if (receivedDataLen >= 65000)
				{
					numToRead = 0;
					printf("Parsing Error\n");
				}
				if (numToRead <= 0)
				{
					state = 0;
				}
				else
				{
					state = 2;  // interrupted packet
				}
				break;
			}
		}
		return(numToRead);
	}
#endif

	int SickScanCommonTcp::readWithTimeout(size_t timeout_ms, char *buffer, int buffer_size, int *bytes_read, bool *exception_occured, bool isBinary)
	{
		// Set up the deadline to the proper timeout, error and delimiters
		deadline_.expires_from_now(boost::posix_time::milliseconds(timeout_ms));
		const char end_delim = static_cast<char>(0x03);
		int dataLen = 0;
		ec_ = boost::asio::error::would_block;
		bytes_transfered_ = 0;

		size_t to_read;

		if (isBinary)
    {
      int numBytes = 0;
      std::vector<unsigned char> recvData = this->recvQueue.pop();
      *bytes_read = recvData.size();
      memcpy(buffer, &(recvData[0]), recvData.size());
      return(ExitSuccess);
#if 1
			boost::asio::async_read(socket_,

				boost::asio::buffer(exampleData, 65536),
				my_completion_handler,
				boost::bind(
					&SickScanCommonTcp::handleRead,
					this,
					boost::asio::placeholders::error,
					boost::asio::placeholders::bytes_transferred
				));
#endif


			do
			{
				io_service_.run_one();
			}
			while (ec_ == boost::asio::error::would_block);

			std::ostream os(&input_buffer_);
			// os << "Hello, World!\n";
			const char *ptr = (const char *)(&(exampleData[0]));
			os.write(ptr,  receivedDataLen);


#if 0
			unsigned char headerData[8] = { 0 };
			// 1236: Die Netzwerkverbindung wurde durch das lokale System


			/*
			boost::asio::async_read(
			s, buffers,
			boost::asio::transfer_all(),
			handler);

			*/


			std::vector<unsigned char> data(8);
			boost::asio::async_read(socket_,
				boost::asio::buffer(data, 8),
				boost::bind(
					&SickScanCommonTcp::handleRead,
					this,
					boost::asio::placeholders::error,
					boost::asio::placeholders::bytes_transferred
				));

			do io_service_.run_one(); while (ec_ == boost::asio::error::would_block);

			for (int i = 4; i < 8; i++)
			{
				int relOffset = i - 4;
				dataLen |= data[i] << (8 * (7 - i));
			}

			dataLen += 1; // wg. CRC
			std::vector<unsigned char> payLoad;
			payLoad.resize(dataLen);

			boost::asio::async_read(socket_,
				boost::asio::buffer(&(payLoad[0]), dataLen),
				boost::asio::transfer_all(),
				boost::bind(
					&SickScanCommonTcp::handleRead,
					this,
					boost::asio::placeholders::error,
					boost::asio::placeholders::bytes_transferred
				)
			);

			do io_service_.run_one(); while (ec_ == boost::asio::error::would_block);

			std::ostream os(&input_buffer_);
			// os << "Hello, World!\n";
			for (int i = 0; i < 8; i++)
			{
				os << data[i];
			}
			for (int i = 0; i < dataLen; i++)
			{
				os << payLoad[i];
			}
#endif
			//	input_buffer_.prepare(8 + dataLen);

			//	input_buffer_.commit(8 + dataLen);

#if 0
			if (ec_ == boost::asio::error::would_block)
			{
				if (bytes_transfered_ > 8)
				{
					for (int i = 0; i < 8; i++)
					{
						if (i < 4)
						{
							if (input_buffer_[i] == 0x02)
							{

								// OK
							}
							else
							{
								ROS_ERROR("sendSOPASCommand: expecting 4-times 0x02, but received: 0x%0x02", headerData[i]);
							}
						}
						else
						{
							int relOffset = i - 4;
							dataLen |= input_buffer_[i] << (8 * (7 - i));
						}
					}
					input_buffer_.consume(8);
					bytes_transfered_ -= 8;
				}
				if (dataLen > 0)
				{
					to_read = bytes_transfered_ > buffer_size - 1 ? buffer_size - 1 : bytes_transfered_;
				}
			}
#endif
		}
		else
		{
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
		}
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
		to_read = bytes_transfered_ > buffer_size - 1 ? buffer_size - 1 : bytes_transfered_;
		size_t i = 0;
		std::istream istr(&input_buffer_);

		if ((to_read > 1000) && (to_read != 0xEE7))
		{
			printf("Stop!");
		}
		if (to_read != 0xEE7)
		{
		// printf("READ %d bytes\n", to_read);
		}
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
	int SickScanCommonTcp::sendSOPASCommand(const char* request, std::vector<unsigned char> * reply, int cmdLen)
	{
#if 0
		if (!socket_.is_open()) {
			ROS_ERROR("sendSOPASCommand: socket not open");
			diagnostics_.broadcast(getDiagnosticErrorCode(), "sendSOPASCommand: socket not open.");
			return ExitError;
		}
#endif
		int sLen = 0;
		int preambelCnt = 0;
		bool cmdIsBinary = false;

		if (request != NULL)
		{
			sLen = cmdLen;
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
				cmdIsBinary = false;
			}
			else
			{
				cmdIsBinary = true;
			}
			int msgLen = 0;
			if (cmdIsBinary == false)
			{
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
#if 1
      m_nw.sendCommandBuffer((UINT8*)request, msgLen);
#else

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
#endif
		}

		// Set timeout in 5 seconds
		const int BUF_SIZE = 1000;
		char buffer[BUF_SIZE];
		int bytes_read;
		if (readWithTimeout(20000, buffer, BUF_SIZE, &bytes_read, 0, cmdIsBinary) == ExitError)
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

	int SickScanCommonTcp::get_datagram(unsigned char* receiveBuffer, int bufferSize, int* actual_length, bool isBinaryProtocol)
	{
    this->setReplyMode(1);
    std::vector<unsigned char> dataBuffer = this->recvQueue.pop();
    long size = dataBuffer.size();
    memcpy(receiveBuffer, &(dataBuffer[0]), size);
    *actual_length = size;

#if 0
		static int cnt = 0;
		char szFileName[255];
		sprintf(szFileName,"/tmp/dg%06d.bin", cnt++);

		FILE *fout;

		fout = fopen(szFileName,"wb");
		if (fout != NULL)
		{
			fwrite(receiveBuffer, size, 1, fout);
			fclose(fout);
		}
#endif
    return ExitSuccess;

		if (!socket_.is_open()) {
			ROS_ERROR("get_datagram: socket not open");
			diagnostics_.broadcast(getDiagnosticErrorCode(), "get_datagram: socket not open.");
			return ExitError;
		}

		/*
		 * Write a SOPAS variable read request to the device.
		 */
		std::vector<unsigned char> reply;

		// Wait at most 5000ms for a new scan
		size_t timeout = 30000;
		bool exception_occured = false;

		char *buffer = reinterpret_cast<char *>(receiveBuffer);

		if (readWithTimeout(timeout, buffer, bufferSize, actual_length, &exception_occured, isBinaryProtocol) != ExitSuccess)
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
