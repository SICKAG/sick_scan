/**
* \file
* \brief Imu Main Handling
* Copyright (C) 2018, Ing.-Buero Dr. Michael Lehning, Hildesheim
* Copyright (C) 2018, SICK AG, Waldkirch
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
*  Last modified: 5th Oct 2018
*
*      Authors:
*         Michael Lehning <michael.lehning@lehning.de>
*
*/

#ifdef _MSC_VER
#define _WIN32_WINNT 0x0501
#pragma warning(disable: 4996)
#pragma warning(disable: 4267)
#endif

#ifndef _MSC_VER


#endif
#include <sick_scan/sick_scan_common_tcp.h>
#include <sick_scan/sick_scan_common.h>
#include <sick_scan/sick_generic_parser.h>
#include <sick_scan/sick_generic_imu.h>
#ifdef _MSC_VER
#include "sick_scan/rosconsole_simu.hpp"
#endif
#include "sensor_msgs/Imu.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include "string"
#include <stdio.h>
#include <stdlib.h>

namespace sick_scan
  {

      bool SickScanImu::isImuDatagram(char *datagram, size_t datagram_length)
        {
        bool ret = false;
        if (this->isImuBinaryDatagram(datagram, datagram_length))
        {
          ret = true;
        } else
        {
          if (this->isImuAsciiDatagram(datagram, datagram_length))
          {
            ret = true;
          }
          else
          {
             if (this->isImuAckDatagram(datagram, datagram_length))
             {
               ret = true;
             }
          }
        }

        return (ret);
        }


      bool SickScanImu::isImuAckDatagram(char *datagram, size_t datagram_length)
        {
        bool isImuMsg = false;
        std::string szKeyWord = "sEA IMUData";
        std::string cmpKeyWord = "";
        int keyWordLen = szKeyWord.length();
        int posTrial[] = {0,1,8};
        for (int iPos = 0; iPos < sizeof(posTrial)/sizeof(posTrial[0]); iPos++)

        if (datagram_length >= (keyWordLen + posTrial[iPos])) // 8 Bytes preheader
        {
          for (int i = 0; i < keyWordLen; i++)
          {
            cmpKeyWord += datagram[i + posTrial[iPos]];
          }
        }
        if (szKeyWord.compare(cmpKeyWord) == 0)
        {
          isImuMsg = true;
        }
        return (isImuMsg);
        }

      /*!
      \brief Checking ASCII diagram for imu message type
      \param datagram: Pointer to datagram data
      \param datagram_length: Number of bytes in datagram
      \return bool flag holding prof result (false -> no ascii imu datagram, true -> ascii imu datagram)
      */
      bool SickScanImu::isImuBinaryDatagram(char *datagram, size_t datagram_length)
        {
        bool isImuMsg = false;
        std::string szKeyWord = "sSN IMUData";
        std::string cmpKeyWord = "";
        int keyWordLen = szKeyWord.length();
        if (datagram_length >= (keyWordLen + 8)) // 8 Bytes preheader
        {
          for (int i = 0; i < keyWordLen; i++)
          {
            cmpKeyWord += datagram[i + 8];
          }
        }
        if (szKeyWord.compare(cmpKeyWord) == 0)
        {
          isImuMsg = true;
        } else
        {
          isImuMsg = false;
        }
        return (isImuMsg);
        }

      /*!
      \brief Parsing Ascii datagram
      \param datagram: Pointer to datagram data
      \param datagram_length: Number of bytes in datagram
      */
      int SickScanImu::parseBinaryDatagram(char *datagram, size_t datagram_length, SickScanImuValue *imuValue)
        {
        int iRet = 0;
        float tmpArr[8] = {0};
        uint32_t timeStamp;
        unsigned char *receiveBuffer = (unsigned char *) datagram;
        if (false == isImuBinaryDatagram(datagram, datagram_length))
        {
          return (-1);
        }

        unsigned char *bufPtr = (unsigned char *) datagram;

        memcpy(&timeStamp, receiveBuffer + 20, 4);
        swap_endian((unsigned char *) &timeStamp, 4);
        int adrOffset = 24;
        for (int i = 0; i < 8; i++)
        {
          memcpy(&(tmpArr[i]), receiveBuffer + adrOffset, 4);
          swap_endian((unsigned char *) (&(tmpArr[i])), 4);
          adrOffset += 4;
        }

        imuValue->QuaternionX(tmpArr[0]);
        imuValue->QuaternionY(tmpArr[1]);
        imuValue->QuaternionZ(tmpArr[2]);
        imuValue->QuaternionW(tmpArr[3]);

        imuValue->QuaternionAccuracy(tmpArr[4]);

        imuValue->AngularVelocityX(tmpArr[5]);
        imuValue->AngularVelocityY(tmpArr[6]);
        imuValue->AngularVelocityZ(tmpArr[7]);


        uint8_t val = receiveBuffer[adrOffset];
        adrOffset++;
        imuValue->AngularVelocityReliability((UINT16) val);
        for (int i = 0; i < 3; i++)
        {
          memcpy(&(tmpArr[i]), receiveBuffer + adrOffset, 4);
          swap_endian((unsigned char *) (&(tmpArr[i])), 4);
          adrOffset += 4;
        }


        imuValue->LinearAccelerationX(tmpArr[0]);
        imuValue->LinearAccelerationY(tmpArr[1]);
        imuValue->LinearAccelerationZ(tmpArr[2]);


        val = receiveBuffer[adrOffset];
        imuValue->LinearAccelerationReliability((UINT16) val);
        return (iRet);
        }

      /*!
      \brief Checking ASCII diagram for imu message type
      \param datagram: Pointer to datagram data
      \param datagram_length: Number of bytes in datagram
      \return bool flag holding prof result (false -> no ascii imu datagram, true -> ascii imu datagram)
      */
      bool SickScanImu::isImuAsciiDatagram(char *datagram, size_t datagram_length)
        {
        bool isImuMsg = false;
        std::string szKeyWord = "sSN IMUData";
        int keyWordLen = szKeyWord.length();
        if (datagram_length >= keyWordLen)
        {

          char *ptr = NULL;
          ptr = strstr(datagram, szKeyWord.c_str());
          if (ptr != NULL)
          {
            int offset = ptr - datagram;
            if ((offset == 0) || (offset == 1)) // should work with 0x02 and without 0x02
            {
              isImuMsg = true;
            }
          }
        }
        return (isImuMsg);
        }

      /*!
      \brief Parsing Ascii datagram
      \param datagram: Pointer to datagram data
      \param datagram_length: Number of bytes in datagram
      */
      int SickScanImu::parseAsciiDatagram(char *datagram, size_t datagram_length, SickScanImuValue *imuValue)
        {
        int exitCode = ExitSuccess;
        bool dumpData = false;
        int verboseLevel = 0;

        // !!!!!
        // verboseLevel = 1;
        int HEADER_FIELDS = 32;
        char *cur_field;
        size_t count;

        // Reserve sufficient space
        std::vector<char *> fields;
        fields.reserve(datagram_length / 2);

        // ----- only for debug output
        std::vector<char> datagram_copy_vec;
        datagram_copy_vec.resize(datagram_length + 1); // to avoid using malloc. destructor frees allocated mem.
        char *datagram_copy = &(datagram_copy_vec[0]);

        if (verboseLevel > 0)
        {
          ROS_WARN("Verbose LEVEL activated. Only for DEBUG.");
        }


        strncpy(datagram_copy, datagram, datagram_length); // datagram will be changed by strtok
        datagram_copy[datagram_length] = 0;

        // ----- tokenize
        count = 0;
        cur_field = strtok(datagram, " ");

        while (cur_field != NULL)
        {
          fields.push_back(cur_field);
          //std::cout << cur_field << std::endl;
          cur_field = strtok(NULL, " ");
        }

        //std::cout << fields[27] << std::endl;

        count = fields.size();

        enum IMU_TOKEN_SEQ // see specification
            {
            IMU_TOKEN_SSN,          // 0: sSN
            IMU_TOKEN_IMUDATA, // 1: LMDradardata
            IMU_TOKEN_TIMESTAMP,  // unsigned long value timestamp
            IMU_TOKEN_QUATERNION_X,
            IMU_TOKEN_QUATERNION_Y,
            IMU_TOKEN_QUATERNION_Z,
            IMU_TOKEN_QUATERNION_W,
            IMU_TOKEN_QUATERNION_ACCURACY, // float value
            IMU_TOKEN_ANGULAR_VELOCITY_X,
            IMU_TOKEN_ANGULAR_VELOCITY_Y,
            IMU_TOKEN_ANGULAR_VELOCITY_Z,
            IMU_TOKEN_ANGULAR_VELOCITY_RELIABILITY, // int value
            IMU_TOKEN_LINEAR_ACCELERATION_X,
            IMU_TOKEN_LINEAR_ACCELERATION_Y,
            IMU_TOKEN_LINEAR_ACCELERATION_Z,
            IMU_TOKEN_LINEAR_ACCELERATION_RELIABILITY, // int value
            IMU_TOKEN_NUM
            };
        for (int i = 0; i < IMU_TOKEN_NUM; i++)
        {
          UINT16 uiValue = 0x00;
          UINT32 udiValue = 0x00;
          unsigned long int uliDummy;
          uliDummy = strtoul(fields[i], NULL, 16);
          float floatDummy;
          switch (i)
          {
            case IMU_TOKEN_TIMESTAMP:
              imuValue->TimeStamp(uliDummy);
              break;
            case IMU_TOKEN_QUATERNION_X:
              memcpy(&floatDummy, &uliDummy, sizeof(float));
              imuValue->QuaternionX(floatDummy);  // following IEEE 754 float convention
              break;
            case IMU_TOKEN_QUATERNION_Y:
              memcpy(&floatDummy, &uliDummy, sizeof(float));
              imuValue->QuaternionY(floatDummy);
              break;
            case IMU_TOKEN_QUATERNION_Z:
              memcpy(&floatDummy, &uliDummy, sizeof(float));
              imuValue->QuaternionZ(floatDummy);
              break;
            case IMU_TOKEN_QUATERNION_W:
              memcpy(&floatDummy, &uliDummy, sizeof(float));
              imuValue->QuaternionW(floatDummy);
              break;

            case IMU_TOKEN_QUATERNION_ACCURACY: // float value
              memcpy(&floatDummy, &uliDummy, sizeof(float));
              imuValue->QuaternionAccuracy(floatDummy);
              break;
            case IMU_TOKEN_ANGULAR_VELOCITY_X:
              memcpy(&floatDummy, &uliDummy, sizeof(float));
              imuValue->AngularVelocityX(floatDummy);
              break;
            case IMU_TOKEN_ANGULAR_VELOCITY_Y:
              memcpy(&floatDummy, &uliDummy, sizeof(float));
              imuValue->AngularVelocityY(floatDummy);
              break;
            case IMU_TOKEN_ANGULAR_VELOCITY_Z:
              memcpy(&floatDummy, &uliDummy, sizeof(float));
              imuValue->AngularVelocityZ(floatDummy);
              break;
            case IMU_TOKEN_ANGULAR_VELOCITY_RELIABILITY:
              uiValue = (UINT16) (0xFFFF & uliDummy);
              imuValue->AngularVelocityReliability(
                      uiValue);  // per definition is a 8 bit value, but we use a 16 bit value
              break;
            case IMU_TOKEN_LINEAR_ACCELERATION_X:
              memcpy(&floatDummy, &uliDummy, sizeof(float));
              imuValue->LinearAccelerationX(floatDummy);
              break;
            case IMU_TOKEN_LINEAR_ACCELERATION_Y:
              memcpy(&floatDummy, &uliDummy, sizeof(float));
              imuValue->LinearAccelerationY(floatDummy);
              break;
            case IMU_TOKEN_LINEAR_ACCELERATION_Z:
              memcpy(&floatDummy, &uliDummy, sizeof(float));
              imuValue->LinearAccelerationZ(floatDummy);
              break;
            case IMU_TOKEN_LINEAR_ACCELERATION_RELIABILITY:
              uiValue = (UINT16) (0xFFFF & uliDummy);
              imuValue->LinearAccelerationReliability(
                      uiValue);  // per definition is a 8 bit value, but we use a 16 bit value
              break;

          }
        }
        return (0);
        }

      void imuParserTest()
        {
        sick_scan::SickScanImu scanImu(NULL);
        sick_scan::SickScanImuValue imuValue;
        //                                             checked with online converter
        //                                             https://www.h-schmidt.net/FloatConverter/IEEE754de.html
        //                                    55570143 0.9998779 -0.0057373047 0.016174316  0.0 0.0 0.002130192             -0.31136206 -0.10777917 9.823472
        std::string imuTestStr = "sSN IMUData 34FEEDF 3F7FF800 BBBC0000 3C848000 00000000 00000000 00000000 3B0B9AB1 00000000 3 BE9F6AD9 BDDCBB53 411D2CF1 0";
        const char imuTestBinStr[] =
                "\x02\x02\x02\x02\x00\x00\x00\x3E"  // 8 Byte Header
                        "\x73\x53\x4E\x20\x49\x4D\x55\x44\x61\x74\x61"  // 11 Byte Keyword
                        "\x20" // 1 Byte Space
                        "\x26\x9E\x05\xEB"  // Offset: 20 Timestamp
                        "\x3F\x7F\xF4\x00"
                        "\xBB\x60\x00\x00"
                        "\x3C\xA2\x80\x00"
                        "\x39\x40\x00\x00"

                        "\x00\x00\x00\x00" // Start-Offset 40

                        "\xBB\x0B\x9A\xB1" // Start-Offset 44
                        "\x3B\x0B\x9A\xB1"
                        "\xBA\x8B\x9A\xB1"
                        "\x03"             // Start-Offset 56
                        "\xBE\xD5\x5F\xC0" // Start-Offset 57
                        "\xBD\x89\x58\x1E"
                        "\x41\x1D\x8A\x24"
                        "\x00"
                        "\xBC";

        char *datagramPtr = (char *) imuTestStr.c_str();
        int datagramLen = imuTestStr.size();

        if (scanImu.isImuAsciiDatagram(datagramPtr, datagramLen))
        {
          scanImu.parseAsciiDatagram(datagramPtr, datagramLen, &imuValue);
        }

        datagramPtr = (char *) imuTestBinStr;
        datagramLen = sizeof(imuTestBinStr) / sizeof(imuTestBinStr[0]);

        if (scanImu.isImuBinaryDatagram(datagramPtr, datagramLen))
        {
          scanImu.parseBinaryDatagram(datagramPtr, datagramLen, &imuValue);
        }

        }

      int SickScanImu::parseDatagram(ros::Time timeStamp, unsigned char *receiveBuffer, int actual_length,
                                     bool useBinaryProtocol)
        {
        int exitCode = ExitSuccess;
        SickScanImuValue imuValue;
        sensor_msgs::Imu imuMsg_;

        if (useBinaryProtocol)
        {
          this->parseBinaryDatagram((char *) receiveBuffer, actual_length, &imuValue);

        } else
        {
          this->parseAsciiDatagram((char *) receiveBuffer, actual_length, &imuValue);
        }

        imuMsg_.header.stamp = timeStamp;
        imuMsg_.header.seq = 0;
        imuMsg_.header.frame_id = "laser"; // todo ...

        imuMsg_.orientation.x = imuValue.QuaternionX();
        imuMsg_.orientation.y = imuValue.QuaternionY();
        imuMsg_.orientation.z = imuValue.QuaternionZ();
        imuMsg_.orientation.w = imuValue.QuaternionW();

        imuMsg_.orientation_covariance[0]  = 1.0;
        imuMsg_.angular_velocity.x = imuValue.AngularVelocityX();
        imuMsg_.angular_velocity.y = imuValue.AngularVelocityY();
        imuMsg_.angular_velocity.z = imuValue.AngularVelocityZ();

        imuMsg_.linear_acceleration.x = imuValue.LinearAccelerationX();
        imuMsg_.linear_acceleration.y = imuValue.LinearAccelerationY();
        imuMsg_.linear_acceleration.z = imuValue.LinearAccelerationZ();

        // setting main diagonal elements of covariance matrix
        // to some meaningful values.
        // see https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/ROS/examples/01.%20Basics/d_IMU/d_IMU.ino
        // as a reference.

        imuMsg_.angular_velocity_covariance[0] = 0.02;
        imuMsg_.angular_velocity_covariance[1] = 0;
        imuMsg_.angular_velocity_covariance[2] = 0;
        imuMsg_.angular_velocity_covariance[3] = 0;
        imuMsg_.angular_velocity_covariance[4] = 0.02;
        imuMsg_.angular_velocity_covariance[5] = 0;
        imuMsg_.angular_velocity_covariance[6] = 0;
        imuMsg_.angular_velocity_covariance[7] = 0;
        imuMsg_.angular_velocity_covariance[8] = 0.02;

        imuMsg_.linear_acceleration_covariance[0] = 0.04;
        imuMsg_.linear_acceleration_covariance[1] = 0;
        imuMsg_.linear_acceleration_covariance[2] = 0;
        imuMsg_.linear_acceleration_covariance[3] = 0;
        imuMsg_.linear_acceleration_covariance[4] = 0.04;
        imuMsg_.linear_acceleration_covariance[5] = 0;
        imuMsg_.linear_acceleration_covariance[6] = 0;
        imuMsg_.linear_acceleration_covariance[7] = 0;
        imuMsg_.linear_acceleration_covariance[8] = 0.04;

        imuMsg_.orientation_covariance[0] = 0.0025;
        imuMsg_.orientation_covariance[1] = 0;
        imuMsg_.orientation_covariance[2] = 0;
        imuMsg_.orientation_covariance[3] = 0;
        imuMsg_.orientation_covariance[4] = 0.0025;
        imuMsg_.orientation_covariance[5] = 0;
        imuMsg_.orientation_covariance[6] = 0;
        imuMsg_.orientation_covariance[7] = 0;
        imuMsg_.orientation_covariance[8] = 0.0025;

        this->commonPtr->imuScan_pub_.publish(imuMsg_);


        return(exitCode);

     }
  }