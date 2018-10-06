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
#define _USE_MATH_DEFINES
#include <math.h>
#include "string"
#include <stdio.h>
#include <stdlib.h>

namespace sick_scan
  {

      /*!
      \brief Parsing Ascii datagram
      \param datagram: Pointer to datagram data
      \param datagram_length: Number of bytes in datagram
      */
      int SickScanImu::parseAsciiDatagram(char* datagram, size_t datagram_length, SickScanImuValue *imuValue)
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

        if (verboseLevel > 0)
        {
          static int cnt = 0;
          char szDumpFileName[255] = {0};
          char szDir[255] = {0};
#ifdef _MSC_VER
          strcpy(szDir, "C:\\temp\\");
#else
          strcpy(szDir, "/tmp/");
#endif
          sprintf(szDumpFileName, "%stmp%06d.bin", szDir, cnt);
          FILE *ftmp;
          ftmp = fopen(szDumpFileName, "wb");
          if (ftmp != NULL)
          {
            fwrite(datagram, datagram_length, 1, ftmp);
            fclose(ftmp);
          }
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


        if (verboseLevel > 0)
        {
          static int cnt = 0;
          char szDumpFileName[255] = {0};
          char szDir[255] = {0};
#ifdef _MSC_VER
          strcpy(szDir, "C:\\temp\\");
#else
          strcpy(szDir, "/tmp/");
#endif
          sprintf(szDumpFileName, "%stmp%06d.txt", szDir, cnt);
          ROS_WARN("Verbose LEVEL activated. Only for DEBUG.");
          FILE *ftmp;
          ftmp = fopen(szDumpFileName, "w");
          if (ftmp != NULL)
          {
            int i;
            for (i = 0; i < count; i++)
            {
              fprintf(ftmp, "%3d: %s\n", i, fields[i]);
            }
            fclose(ftmp);
          }
          cnt++;
        }


        enum IMU_TOKEN_SEQ
            {
            IMU_TOKEN_SSN,          // 0: sSN
            IMU_TOKEN_IMUDATA, // 1: LMDradardata
            IMU_TOKEN_QUATERNION_X,
            IMU_TOKEN_QUATERNION_Y,
            IMU_TOKEN_QUATERNION_Z,
            IMU_TOKEN_QUATERNION_W,
            IMU_TOKEN_QUATERNION_ACCURACY,
            IMU_TOKEN_ANGULAR_VELOCITY_X,
            IMU_TOKEN_ANGULAR_VELOCITY_Y,
            IMU_TOKEN_ANGULAR_VELOCITY_Z,
            IMU_TOKEN_ANGULAR_VELOCITY_RELIABILITY,
            IMU_TOKEN_LINEAR_ACCELERATION_X,
            IMU_TOKEN_LINEAR_ACCELERATION_Y,
            IMU_TOKEN_LINEAR_ACCELERATION_Z,
            IMU_TOKEN_LINEAR_ACCELERATION_RELIABILITY,
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
            case IMU_TOKEN_QUATERNION_X:
              swap_endian((unsigned char *) &uliDummy, 4);
              memcpy(&floatDummy, &uliDummy, sizeof(float));
              imuValue->QuaternionX(floatDummy);
              break;
            case IMU_TOKEN_QUATERNION_Y:
              swap_endian((unsigned char *) &uliDummy, 4);
              memcpy(&floatDummy, &uliDummy, sizeof(float));
              imuValue->QuaternionY(floatDummy);
              break;
            case IMU_TOKEN_QUATERNION_Z:
              swap_endian((unsigned char *) &uliDummy, 4);
              memcpy(&floatDummy, &uliDummy, sizeof(float));
              imuValue->QuaternionZ(floatDummy);
              break;
            case IMU_TOKEN_QUATERNION_W:
              swap_endian((unsigned char *) &uliDummy, 4);
              memcpy(&floatDummy, &uliDummy, sizeof(float));
              imuValue->QuaternionW(floatDummy);
              break;
          }
        }
        }
  }