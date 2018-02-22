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
*         Michael Lehning <michael.lehning@lehning.de>
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

	SickScanCommonTcp::SickScanCommonTcp(const std::string &hostname, const std::string &port, int &timelimit, SickGenericParser* parser, char cola_dialect_id)
		:
		SickScanCommon(parser),
		socket_(io_service_),
		deadline_(io_service_),
		hostname_(hostname),
		port_(port),
		timelimit_(timelimit)
	{

		if ((cola_dialect_id == 'a') || (cola_dialect_id == 'A'))
		{
			this->setProtocolType(CoLa_A);
		}

		if ((cola_dialect_id == 'b') || (cola_dialect_id == 'B'))
		{
			this->setProtocolType(CoLa_B);
		}

		assert(this->getProtocolType() != CoLa_Unknown);

		m_numberOfBytesInReceiveBuffer = 0;
		m_alreadyReceivedBytes = 0;
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

#if 0
	void SickScanCommonTcp::setProtocolType(char cola_dialect_id)
	{
		if ((cola_dialect_id == 'a') || (cola_dialect_id == 'A'))
		{
			this->m_protocol = CoLa_A;
		}
		else
		{
			this->m_protocol = CoLa_B;
		}
	}
#endif


	//
	// Look for 23-frame (STX/ETX) in receive buffer.
	// Move frame to start of buffer
	//
	// Return: 0 : No (complete) frame found
	//        >0 : Frame length
	//
	SopasEventMessage SickScanCommonTcp::findFrameInReceiveBuffer()
	{
		UINT32 frameLen = 0;
		UINT32 i;

		// Depends on protocol...
		if (getProtocolType() == CoLa_A)
		{
			//
			// COLA-A
			//
			// Must start with STX (0x02)
			if (m_receiveBuffer[0] != 0x02)
			{
				// Look for starting STX (0x02)
				for (i = 1; i < m_numberOfBytesInReceiveBuffer; i++)
				{
					if (m_receiveBuffer[i] == 0x02)
					{
						break;
					}
				}

				// Found beginning of frame?
				if (i >= m_numberOfBytesInReceiveBuffer)
				{
					// No start found, everything can be discarded
					m_numberOfBytesInReceiveBuffer = 0; // Invalidate buffer
					return SopasEventMessage(); // No frame found
				}

				// Move frame start to index 0
				UINT32 newLen = m_numberOfBytesInReceiveBuffer - i;
				memmove(&(m_receiveBuffer[0]), &(m_receiveBuffer[i]), newLen);
				m_numberOfBytesInReceiveBuffer = newLen;
			}

			// Look for ending ETX (0x03)
			for (i = 1; i < m_numberOfBytesInReceiveBuffer; i++)
			{
				if (m_receiveBuffer[i] == 0x03)
				{
					break;
				}
			}

			// Found end?
			if (i >= m_numberOfBytesInReceiveBuffer)
			{
				// No end marker found, so it's not a complete frame (yet)
				return SopasEventMessage(); // No frame found
			}

			// Calculate frame length in byte
			frameLen = i + 1;

			return SopasEventMessage(m_receiveBuffer, CoLa_A, frameLen);
		}
		else if (getProtocolType() == CoLa_B)
		{
			UINT32 magicWord;
			UINT32 payloadlength;

			if (m_numberOfBytesInReceiveBuffer < 4)
			{
				return SopasEventMessage();
			}
			UINT16 pos = 0;
			magicWord = colab::getIntegerFromBuffer<UINT32>(m_receiveBuffer, pos);
			if (magicWord != 0x02020202)
			{
				// Look for starting STX (0x02020202)
				for (i = 1; i <= m_numberOfBytesInReceiveBuffer - 4; i++)
				{
					pos = i; // this is needed, as the position value is updated by getIntegerFromBuffer
					magicWord = colab::getIntegerFromBuffer<UINT32>(m_receiveBuffer, pos);
					if (magicWord == 0x02020202)
					{
						// found magic word
						break;
					}
				}

				// Found beginning of frame?
				if (i > m_numberOfBytesInReceiveBuffer - 4)
				{
					// No start found, everything can be discarded
					m_numberOfBytesInReceiveBuffer = 0; // Invalidate buffer
					return SopasEventMessage(); // No frame found
				}
				else
				{
					// Move frame start to index
					UINT32 bytesToMove = m_numberOfBytesInReceiveBuffer - i;
					memmove(&(m_receiveBuffer[0]), &(m_receiveBuffer[i]), bytesToMove); // payload+magic+length+s+checksum
					m_numberOfBytesInReceiveBuffer = bytesToMove;
				}
			}

			// Pruefe Laenge des Pufferinhalts
			if (m_numberOfBytesInReceiveBuffer < 9)
			{
				// Es sind nicht genug Daten fuer einen Frame
				printInfoMessage("SickScanCommonNw::findFrameInReceiveBuffer: Frame cannot be decoded yet, only " +
					::toString(m_numberOfBytesInReceiveBuffer) + " bytes in the buffer.", m_beVerbose);
				return SopasEventMessage();
			}

			// Read length of payload
			pos = 4;
			payloadlength = colab::getIntegerFromBuffer<UINT32>(m_receiveBuffer, pos);
			printInfoMessage("SickScanCommonNw::findFrameInReceiveBuffer: Decoded payload length is " + ::toString(payloadlength) + " bytes.", m_beVerbose);

			// Ist die Datenlaenge plausibel und wuede in den Puffer passen?
			if (payloadlength > (sizeof(m_receiveBuffer) - 9))
			{
				// magic word + length + checksum = 9
				printWarning("SickScanCommonNw::findFrameInReceiveBuffer: Frame too big for receive buffer. Frame discarded with length:"
					+ ::toString(payloadlength) + ".");
				m_numberOfBytesInReceiveBuffer = 0;
				return SopasEventMessage();
			}
			if ((payloadlength + 9) > m_numberOfBytesInReceiveBuffer)
			{
				// magic word + length + s + checksum = 10
				printInfoMessage("SickScanCommonNw::findFrameInReceiveBuffer: Frame not complete yet. Waiting for the rest of it (" +
					::toString(payloadlength + 9 - m_numberOfBytesInReceiveBuffer) + " bytes missing).", m_beVerbose);
				return SopasEventMessage(); // frame not complete
			}

			// Calculate the total frame length in bytes: Len = Frame (9 bytes) + Payload
			frameLen = payloadlength + 9;

			//
			// test checksum of payload
			//
			UINT8 temp = 0;
			UINT8 temp_xor = 0;
			UINT8 checkSum;

			// Read original checksum
			pos = frameLen - 1;
			checkSum = colab::getIntegerFromBuffer<UINT8>(m_receiveBuffer, pos);

			// Erzeuge die Pruefsumme zum Vergleich
			for (UINT16 i = 8; i < (frameLen - 1); i++)
			{
				pos = i;
				temp = colab::getIntegerFromBuffer<UINT8>(m_receiveBuffer, pos);
				temp_xor = temp_xor ^ temp;
			}

			// Vergleiche die Pruefsummen
			if (temp_xor != checkSum)
			{
				printWarning("SickScanCommonNw::findFrameInReceiveBuffer: Wrong checksum, Frame discarded.");
				m_numberOfBytesInReceiveBuffer = 0;
				return SopasEventMessage();
			}

			return SopasEventMessage(m_receiveBuffer, CoLa_B, frameLen);
		}

		// Return empty frame
		return SopasEventMessage();
	}



	/**
 * Read callback. Diese Funktion wird aufgerufen, sobald Daten auf der Schnittstelle
 * hereingekommen sind.
 */

	void SickScanCommonTcp::processFrame(SopasEventMessage& frame)
	{

		if (getProtocolType() == CoLa_A)
		{
			printInfoMessage("SickScanCommonNw::processFrame: Calling processFrame_CoLa_A() with " + ::toString(frame.size()) + " bytes.", m_beVerbose);
			// processFrame_CoLa_A(frame);
		}
		else if (getProtocolType() == CoLa_B)
		{
			printInfoMessage("SickScanCommonNw::processFrame: Calling processFrame_CoLa_B() with " + ::toString(frame.size()) + " bytes.", m_beVerbose);
			// processFrame_CoLa_B(frame);
		}

		// Push frame to recvQueue
		recvQueue.push(std::vector<unsigned char>(frame.getRawData(), frame.getRawData() + frame.size()));

	}
	void SickScanCommonTcp::readCallbackFunction(UINT8* buffer, UINT32& numOfBytes)
	{
		bool beVerboseHere = false;
		printInfoMessage("SickScanCommonNw::readCallbackFunction(): Called with " + toString(numOfBytes) + " available bytes.", beVerboseHere);

		ScopedLock lock(&m_receiveDataMutex); // Mutex for access to the input buffer
		UINT32 remainingSpace = sizeof(m_receiveBuffer) - m_numberOfBytesInReceiveBuffer;
		UINT32 bytesToBeTransferred = numOfBytes;
		if (remainingSpace < numOfBytes)
		{
			bytesToBeTransferred = remainingSpace;
			// printWarning("SickScanCommonNw::readCallbackFunction(): Input buffer space is to small, transferring only " +
			//              ::toString(bytesToBeTransferred) + " of " + ::toString(numOfBytes) + " bytes.");
		}
		else
		{
			// printInfoMessage("SickScanCommonNw::readCallbackFunction(): Transferring " + ::toString(bytesToBeTransferred) +
			//                   " bytes from TCP to input buffer.", beVerboseHere);
		}

		if (bytesToBeTransferred > 0)
		{
			// Data can be transferred into our input buffer
			memcpy(&(m_receiveBuffer[m_numberOfBytesInReceiveBuffer]), buffer, bytesToBeTransferred);
			m_numberOfBytesInReceiveBuffer += bytesToBeTransferred;

			UINT32 size = 0;

			while (1)
			{
				// Now work on the input buffer until all received datasets are processed
				SopasEventMessage frame = findFrameInReceiveBuffer();

				size = frame.size();
				if (size == 0)
				{
					// Framesize = 0: There is no valid frame in the buffer. The buffer is either empty or the frame
					// is incomplete, so leave the loop
					printInfoMessage("SickScanCommonNw::readCallbackFunction(): No complete frame in input buffer, we are done.", beVerboseHere);

					// Leave the loop
					break;
				}
				else
				{
					// A frame was found in the buffer, so process it now.
					printInfoMessage("SickScanCommonNw::readCallbackFunction(): Processing a frame of length " + ::toString(frame.size()) + " bytes.", beVerboseHere);
					processFrame(frame);
					UINT32 bytesToMove = m_numberOfBytesInReceiveBuffer - size;
					memmove(&(m_receiveBuffer[0]), &(m_receiveBuffer[size]), bytesToMove); // payload+magic+length+s+checksum
					m_numberOfBytesInReceiveBuffer = bytesToMove;

				}
			}
		}
		else
		{
			// There was input data from the TCP interface, but our input buffer was unable to hold a single byte.
			// Either we have not read data from our buffer for a long time, or something has gone wrong. To re-sync,
			// we clear the input buffer here.
			m_numberOfBytesInReceiveBuffer = 0;
		}
	}

	void SickScanCommonTcp::readCallbackFunctionOld(UINT8* buffer, UINT32& numOfBytes)
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

		// starting with 0x02 - but no magic word -> ASCII-Command-Reply
		if ((numOfBytes < 2) && (m_alreadyReceivedBytes == 0))
		{
			return;  // ultra short message (only 1 byte) must be nonsense 
		}
		if ((buffer[0] == 0x02) && (buffer[1] != 0x02)) // no magic word, but received initial 0x02 -> guess Ascii reply
		{
			if (numOfBytes > 0)
			{
				// check last character of message - must be 0x03 
				char lastChar = buffer[numOfBytes - 1];  // check last for 0x03
				if (lastChar == 0x03)  
				{
					memcpy(m_packetBuffer, buffer, numOfBytes);
					m_alreadyReceivedBytes = numOfBytes;
					recvQueue.push(std::vector<unsigned char>(m_packetBuffer, m_packetBuffer + numOfBytes));
					m_alreadyReceivedBytes = 0;
				}
				else
				{

					ROS_WARN("Dropping packages???\n");
					FILE *fout = fopen("/tmp/package.bin", "wb");
					if (fout != NULL)
					{
						fwrite(m_packetBuffer, 1, numOfBytes, fout);
						fclose(fout);
					}
				}
			}
		}

		if ((numOfBytes < 9) && (m_alreadyReceivedBytes == 0))
		{
			return;
		}


		// check magic word for cola B
		if ((m_alreadyReceivedBytes > 0) || (buffer[0] == 0x02 && buffer[1] == 0x02 && buffer[2] == 0x02 && buffer[3] == 0x02))
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
				if (strncmp((char *)(buffer + 8), packetKeyWord, strlen(packetKeyWord)) != 0)
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
		}
	}


	int SickScanCommonTcp::init_device()
	{
		int portInt;
		sscanf(port_.c_str(), "%d", &portInt);
		m_nw.init(hostname_, portInt, disconnectFunctionS, (void*)this);
		m_nw.setReadCallbackFunction(readCallbackFunctionS, (void*)this);
		m_nw.connect();
		return ExitSuccess;
	}

	int SickScanCommonTcp::close_device()
	{
		m_nw.disconnect();
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


	int SickScanCommonTcp::readWithTimeout(size_t timeout_ms, char *buffer, int buffer_size, int *bytes_read, bool *exception_occured, bool isBinary)
	{
		// Set up the deadline to the proper timeout, error and delimiters
		deadline_.expires_from_now(boost::posix_time::milliseconds(timeout_ms));
		const char end_delim = static_cast<char>(0x03);
		int dataLen = 0;
		ec_ = boost::asio::error::would_block;
		bytes_transfered_ = 0;

		size_t to_read;

		int numBytes = 0;
		// Polling - should be changed to condition variable in the future
		int waitingTimeInMs = 10;
		int i;
		for (i = 0; i < timeout_ms; i += waitingTimeInMs)
		{
			if (false == this->recvQueue.isQueueEmpty())
			{
				break;
			}
			boost::this_thread::sleep(boost::posix_time::milliseconds(waitingTimeInMs));
		}
		if (i >= timeout_ms)
		{
			ROS_ERROR("no answer received after %zu ms. Maybe sopas mode is wrong.\n",timeout_ms);
			return(ExitError);
		}
		boost::condition_variable cond_;
		std::vector<unsigned char> recvData = this->recvQueue.pop();
		*bytes_read = recvData.size();
		memcpy(buffer, &(recvData[0]), recvData.size());
		return(ExitSuccess);
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
		// !!!
		if (readWithTimeout(getReadTimeOutInMs(), buffer, BUF_SIZE, &bytes_read, 0, cmdIsBinary) == ExitError)
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
		const int maxWaitInMs = 2000;
		std::vector<unsigned char> dataBuffer;
#if 0 // prepared for reconnect
		bool retVal = this->recvQueue.waitForIncomingObject(maxWaitInMs);
		if (retVal == false)
		{
			ROS_WARN("Timeout during waiting of new datagram");
			return ExitError;
		}
		else
		{
			dataBuffer = this->recvQueue.pop();
		}
#endif
		dataBuffer = this->recvQueue.pop();

		long size = dataBuffer.size();
		memcpy(receiveBuffer, &(dataBuffer[0]), size);
		*actual_length = size;

#if 0
		static int cnt = 0;
		char szFileName[255];
		sprintf(szFileName, "/tmp/dg%06d.bin", cnt++);

		FILE *fout;

		fout = fopen(szFileName, "wb");
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
