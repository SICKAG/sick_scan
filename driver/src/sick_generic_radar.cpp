/**
* \file
* \brief Radar Main Handling
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
*  Last modified: 29th May 2018
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
#include <sick_scan/sick_generic_parser.h>
#include <sick_scan/sick_generic_radar.h>
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
	int16_t getShortValue(std::string str)
	{
		int val = 0;
		if (1 == sscanf(str.c_str(), "%x", &val))
		{

		}
		else
		{
			ROS_WARN("Problems parsing %s\n", str.c_str());
		}
		return(val);

	}

	int getHexValue(std::string str)
	{
		int val = 0;
		if (1 == sscanf(str.c_str(), "%x", &val))
		{

		}
		else
		{
			ROS_WARN("Problems parsing %s\n", str.c_str());
		}
		return(val);

	}

	float getFloatValue(std::string str)
	{
		float tmpVal = 0.0;
		unsigned char *ptr;
		ptr = (unsigned char *)(&tmpVal);
		int strLen = str.length();
		if (strLen < 8)
		{
		}
		else
		{
			for (int i = 0; i < 4; i++)
			{
				std::string dummyStr = "";
				dummyStr += str[i * 2];
				dummyStr += str[i * 2 + 1];
				int val = getHexValue(dummyStr);
				unsigned char ch = (0xFF & val);
				ptr[3 - i] = ch;
			}
		}
		return(tmpVal);
	}

	/*!
	\brief Parsing Ascii datagram
	\param datagram: Pointer to datagram data
	\param datagram_length: Number of bytes in datagram
	\param config: Pointer to Configdata
	\param msg: Holds result of Parsing
	\param numEchos: Number of DIST-blocks found in message
	\param echoMask: Mask corresponding to DIST-block-identifier
	\return set_range_max
	*/
	int SickScanRadar::parseAsciiDatagram(char* datagram, size_t datagram_length, std::vector<SickScanRadarObject> &objectList, std::vector<SickScanRadarRawTarget> &rawTargetList)
	{
		int exitCode = ExitSuccess;
		// echoMask introduced to get a workaround for cfg bug using MRS1104
		ros::NodeHandle tmpParam("~");
		bool dumpData = false;
		int verboseLevel = 0;
		tmpParam.getParam("verboseLevel", verboseLevel);

		// !!!!!

		verboseLevel = 1;
		int HEADER_FIELDS = 32;
		char* cur_field;
		size_t count;

		// Reserve sufficient space
		std::vector<char *> fields;
		fields.reserve(datagram_length / 2);

		// ----- only for debug output
		std::vector<char> datagram_copy_vec;
		datagram_copy_vec.resize(datagram_length + 1); // to avoid using malloc. destructor frees allocated mem.
		char* datagram_copy = &(datagram_copy_vec[0]);

		if (verboseLevel > 0) {
			ROS_WARN("Verbose LEVEL activated. Only for DEBUG.");
		}

		if (verboseLevel > 0)
		{
			static int cnt = 0;
			char szDumpFileName[255] = { 0 };
			char szDir[255] = { 0 };
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
			char szDumpFileName[255] = { 0 };
			char szDir[255] = { 0 };
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

		std::vector<std::string> keyWordList;
#define DIST1_KEYWORD "DIST1"
#define AZMT1_KEYWORD "AZMT1"
#define VRAD1_KEYWORD "VRAD1"
#define AMPL1_KEYWORD "AMPL1"
#define MODE1_KEYWORD "MODE1"

#define P3DX1_KEYWORD "P3DX1"
#define P3DY1_KEYWORD "P3DY1"
#define V3DX1_KEYWORD "V3DX1"
#define V3DY1_KEYWORD "V3DY1"
#define OBJLEN_KEYWORD "OBLE1"
#define OBJID_KEYWORD "OBID1"

		const int RAWTARGET_LOOP = 0;
		const int OBJECT_LOOP = 1;

		// std::vector<SickScanRadarRawTarget> rawTargets;
		// std::vector<SickScanRadarObject> objectList;

		for (int iLoop = 0; iLoop < 2; iLoop++)
		{
			keyWordList.clear();
			switch (iLoop)
			{
			case RAWTARGET_LOOP:
				keyWordList.push_back(DIST1_KEYWORD);
				keyWordList.push_back(AZMT1_KEYWORD);
				keyWordList.push_back(VRAD1_KEYWORD);
				keyWordList.push_back(AMPL1_KEYWORD);
				keyWordList.push_back(MODE1_KEYWORD);
				break;
			case OBJECT_LOOP:
				keyWordList.push_back(P3DX1_KEYWORD);
				keyWordList.push_back(P3DY1_KEYWORD);
				keyWordList.push_back(V3DX1_KEYWORD);
				keyWordList.push_back(V3DY1_KEYWORD);
				keyWordList.push_back(OBJLEN_KEYWORD);
				keyWordList.push_back(OBJID_KEYWORD);
				break;
			}
			std::vector<int> keyWordPos;
			std::vector<float> keyWordScale;
			std::vector<float> keyWordOffset;
			keyWordPos.resize(keyWordList.size());
			keyWordScale.resize(keyWordList.size());
			for (int i = 0; i < keyWordPos.size(); i++)
			{
				keyWordPos[i] = -1;
			}
			int numKeyWords = keyWordPos.size();
			for (int i = 0; i < fields.size(); i++)
			{
				for (int j = 0; j < keyWordList.size(); j++)
				{
					if (strcmp(fields[i], keyWordList[j].c_str()) == 0)
					{
						keyWordPos[j] = i;
					}
				}
			}

			bool entriesNumOk = true;
			int  entriesNum = 0;
			if (keyWordPos[0] == -1)
			{
				entriesNumOk = false;
			}
			else
			{

				entriesNum = getHexValue(fields[keyWordPos[0] + 3]);
				for (int i = 0; i < numKeyWords; i++)
				{
					if (keyWordPos[i] == -1)
					{
						entriesNumOk = false;
						ROS_WARN("Missing keyword %s but first keyword found.\n", keyWordList[i].c_str());
						entriesNumOk = false;
					}
					else
					{
						int entriesNumTmp = getHexValue(fields[keyWordPos[i] + 3]);
						if (entriesNumTmp != entriesNum)
						{
							ROS_WARN("Number of items for keyword %s differs from number of items for %s\n.",
								keyWordList[i].c_str(), keyWordList[0].c_str());
							entriesNumOk = false;
						}
					}
				}
			}

			if (true == entriesNumOk)
			{


				for (int i = 0; i < numKeyWords; i++)
				{
					int scaleLineIdx = keyWordPos[i] + 1;
					std::string token = fields[scaleLineIdx];
					keyWordScale[i] = getFloatValue(token);
					printf("Keyword: %-6s %8.3lf\n", keyWordList[i].c_str(), keyWordScale[i]);
				}

				switch (iLoop)
				{
				case RAWTARGET_LOOP:
				{
					rawTargetList.resize(entriesNum);
					break;
				}
				case OBJECT_LOOP:
				{
					objectList.resize(entriesNum);
					break;
				}
				}
				for (int i = 0; i < entriesNum; i++)
				{
					switch (iLoop)
					{
					case RAWTARGET_LOOP:
					{
						float dist = 0.0;
						float azimuth = 0.0;
						float ampl = 0.0;
						float vrad = 0.0;
						int mode = 0;
						for (int j = 0; j < numKeyWords; j++)
						{
							int dataRowIdx = keyWordPos[j] + 4 + i;
							std::string token = keyWordList[j];
							if (token.compare(DIST1_KEYWORD) == 0)
							{
								int distRaw = getHexValue(fields[dataRowIdx]);
								dist = (float)distRaw * keyWordScale[j] * 0.001;
							}
							if (token.compare(AZMT1_KEYWORD) == 0)
							{
								int azimuthRaw = getShortValue(fields[dataRowIdx]);
								azimuth = (float)azimuthRaw * keyWordScale[j];
							}
							if (token.compare(VRAD1_KEYWORD) == 0)
							{
								int vradRaw = getShortValue(fields[dataRowIdx]);
								vrad = (float)vradRaw * keyWordScale[j];
							}
							if (token.compare(MODE1_KEYWORD) == 0)
							{
								int modeRaw = getHexValue(fields[dataRowIdx]);
								mode = (int)(modeRaw * keyWordScale[j] + 0.5);
							}
							if (token.compare(AMPL1_KEYWORD) == 0)
							{
								int amplRaw = getHexValue(fields[dataRowIdx]);
								ampl = (int)(amplRaw * keyWordScale[j] + 0.5);
							}
						}
						rawTargetList[i].Dist(dist);
						rawTargetList[i].Ampl(ampl);
						rawTargetList[i].Azimuth(azimuth);
						rawTargetList[i].Mode(mode);
						rawTargetList[i].Vrad(vrad);

					}
					break;
					case OBJECT_LOOP:
					{
						float px = 0.0;
						float py = 0.0;
						float vx = 0.0;
						float vy = 0.0;
						float objLen = 0.0;
						int objId = 0;

						for (int j = 0; j < numKeyWords; j++)
						{
							int dataRowIdx = keyWordPos[j] + 4 + i;
							std::string token = keyWordList[j];
							if (token.compare(P3DX1_KEYWORD) == 0)
							{
								int pxRaw = getShortValue(fields[dataRowIdx]);
								px = (float)pxRaw * keyWordScale[j] * 0.001;
							}
							if (token.compare(P3DY1_KEYWORD) == 0)
							{
								int pyRaw = getShortValue(fields[dataRowIdx]);
								py = (float)pyRaw * keyWordScale[j] * 0.001;
							}
							if (token.compare(V3DX1_KEYWORD) == 0)
							{
								int vxRaw = getShortValue(fields[dataRowIdx]);
								vx = (float)vxRaw * keyWordScale[j];
							}
							if (token.compare(V3DY1_KEYWORD) == 0)
							{
								int vyRaw = getShortValue(fields[dataRowIdx]);
								vy = (float)vyRaw * keyWordScale[j];
							}
							if (token.compare(OBJLEN_KEYWORD) == 0)
							{
								int objLenRaw = getShortValue(fields[dataRowIdx]);
								objLen = (float)objLenRaw * keyWordScale[j];
							}
							if (token.compare(OBJID_KEYWORD) == 0)
							{
								int objIdRaw = getHexValue(fields[dataRowIdx]);
								objId = (int)(objIdRaw * keyWordScale[j] + 0.5);
							}
						}

						objectList[i].ObjId(objId);
						objectList[i].ObjLength(objLen);
						objectList[i].P3Dx(px);
						objectList[i].P3Dy(py);
						objectList[i].V3Dx(vx);
						objectList[i].V3Dy(vy);
					}
					break;
					}
				}
			}
			// Now parsing the entries
		}

		return(exitCode);
	}

	void SickScanRadar::simulateAsciiDatagram(unsigned char * receiveBuffer, int* actual_length)
	{
		std::string header = "\x2sSN LMDradardata 1 1 112F6E9 0 0 533A 0 0 28C7DDDC 0 0 0 0 CB00 780 1 0 0 ";
		int channel16BitCnt = 4;
		// Faktoren fuer Rohziele: 40.0 0.16 0.04 1.00 1.00
		float rawTargetFactorList[] = { 40.0f, 0.16f, 0.04f, 1.00f, 1.00f };
		float objectFactorList[] = { 64.0f, 64.0f, 0.1f, 0.1f, 0.75f, 1.0f };

		std::string dist1_intro = "DIST1 42200000 00000000";
		std::string azmt1_intro = "AZMT1 3E23D70A 00000000";
		std::string vrad1_intro = "VRAD1 3D23D70A 00000000";
		std::string ampl1_intro = "AMPL1 3F800000 00000000";
		int rawTargetChannel16BitCnt = 4;
		int rawTargetChannel8BitCnt = 1;
		std::string mode1_intro = "MODE1 3F800000 00000000";

		std::string pdx1_intro = "P3DX1 42800000 00000000";
		std::string pdy1_intro = "P3DY1 42800000 00000000";
		std::string v3dx_intro = "V3DX1 3DCCCCCD 00000000";
		std::string v3dy_intro = "V3DY1 3DCCCCCD 00000000";
		std::string oblen_intro = "OBLE1 3F400000 00000000";


		std::string obid_intro = "OBID1 3F800000 00000000";



		std::string trailer = "0 0 0 0 0 0\x3";

		std::vector<std::string> channel16BitID;
		std::vector<std::string> channel8BitID;
		channel16BitID.push_back(dist1_intro);
		channel16BitID.push_back(azmt1_intro);
		channel16BitID.push_back(vrad1_intro);
		channel16BitID.push_back(ampl1_intro);

		channel16BitID.push_back(pdx1_intro);
		channel16BitID.push_back(pdy1_intro);
		channel16BitID.push_back(v3dx_intro);
		channel16BitID.push_back(v3dy_intro);
		channel16BitID.push_back(oblen_intro);



		channel8BitID.push_back(mode1_intro);
		channel8BitID.push_back(obid_intro);

		int channel8BitCnt = channel8BitID.size();
		int objectChannel16BitCnt = 5;
		channel16BitCnt = channel16BitID.size();
		float x = 20.0;
		float speed = 50.0; // [m/s]
		std::vector<SickScanRadarRawTarget> rawTargetList;
		for (float y = -20; y <= 20.0; y += 2.0)
		{
			SickScanRadarRawTarget rawTarget;
			float azimuth = atan2(y, x);
			float dist = sqrt(x*x + y*y);
			float vrad = speed * sin(azimuth);  // speed in y direction 
			float mode = 0;
			float ampl = 50.0;
			rawTarget.Ampl(ampl);
			rawTarget.Mode(mode);
			rawTarget.Dist(dist);
			rawTarget.Azimuth(azimuth);
			rawTarget.Vrad(vrad);
			rawTargetList.push_back(rawTarget);
		}

		std::vector<SickScanRadarObject> objectList;

		int objId = 0;
		for (float x = 20; x <= 100.0; x += 10.0)
		{
			SickScanRadarObject vehicle;
			float y = 0.0;
			for (int iY = -1; iY <= 1; iY += 2)
			{
				y = iY * 2.0;

				vehicle.P3Dx(x * 1000.0);
				vehicle.P3Dy(y * 1000.0);
				vehicle.V3Dx(y * 10.0f); // +/- 20 m/s
				vehicle.V3Dy(0.1f); // just for testing
				vehicle.ObjLength(6.0f + y);
				vehicle.ObjId(objId++);
				objectList.push_back(vehicle);
			}


		}

		char szDummy[255] = { 0 };
		std::string resultStr;
		resultStr += header;
		sprintf(szDummy, "%x ", channel16BitCnt);
		resultStr += szDummy;
		for (int i = 0; i < channel16BitCnt; i++)
		{
			resultStr += channel16BitID[i];
			int valNum = rawTargetList.size();
			bool processRawTarget = true;
			if (i < rawTargetChannel16BitCnt)
			{
				valNum = rawTargetList.size();
			}
			else
			{
				processRawTarget = false;
				valNum = objectList.size();
			}

			sprintf(szDummy, " %x ", valNum);
			resultStr += szDummy;
			float val = 0.0;
			for (int j = 0; j < valNum; j++)
			{
				switch (i)
				{
				case 0: val = 1000.0 * rawTargetList[j].Dist(); break;
				case 1: val = 1.0 / deg2rad * rawTargetList[j].Azimuth(); break;
				case 2: val = rawTargetList[j].Vrad(); break;
				case 3: val = rawTargetList[j].Ampl(); break;
				case 4: val = objectList[j].P3Dx(); break;
				case 5: val = objectList[j].P3Dy(); break;
				case 6: val = objectList[j].V3Dx(); break;
				case 7: val = objectList[j].V3Dy(); break;
				case 8: val = objectList[j].ObjLength(); break;
				}


				if (processRawTarget)
				{
					val /= rawTargetFactorList[i];
				}
				else
				{
					int idx = i - rawTargetChannel16BitCnt;
					val /= objectFactorList[idx];
				}

				int16_t shortVal = (int16_t)(val + 0.5);

				sprintf(szDummy, "%08x", (int)val);
				strcpy(szDummy, szDummy + 4);  // remove first 4 digits due to integer / short
				resultStr += szDummy;
				resultStr += " ";
			}
		}

		sprintf(szDummy, "%x ", channel8BitCnt);
		resultStr += szDummy;
		int valNum = rawTargetList.size();
		for (int i = 0; i < channel8BitCnt; i++)
		{
			resultStr += channel8BitID[i];
			float val = 0.0;
			bool processRawTarget = true;
			if (i < rawTargetChannel8BitCnt)
			{
				valNum = rawTargetList.size();
			}
			else
			{
				processRawTarget = false;
				valNum = objectList.size();
			}
			sprintf(szDummy, " %x ", valNum);
			resultStr += szDummy;
			for (int j = 0; j < valNum; j++)
			{
				switch (i)
				{
				case 0: val = rawTargetList[j].Mode(); break;
				case 1: val = objectList[j].ObjId(); break;
				}

				int offset = 0;
				if (processRawTarget)
				{
					offset = rawTargetChannel16BitCnt;
					val /= rawTargetFactorList[i + offset];
				}
				else
				{
					offset = objectChannel16BitCnt;
					int idx = i - rawTargetChannel8BitCnt;
					val /= objectFactorList[idx + offset];
				}
				int8_t shortVal = (int16_t)(val + 0.5);

				sprintf(szDummy, "%08x", (int)val);
				strcpy(szDummy, szDummy + 6);  // remove first 6 digits due to integer / short
				resultStr += szDummy;
				resultStr += " ";
			}
		}

		resultStr += trailer;

			*actual_length = resultStr.length();
		strcpy((char *)receiveBuffer, resultStr.c_str());
		}

	void SickScanRadar::simulateAsciiDatagramFromFile(unsigned char * receiveBuffer, int* pActual_length)
	{
		static int callCnt = 0;
		std::string fileName = "C:\\temp\\scanTrain\\tmp000791.txt";
		FILE *fin;
		char szLine[1024] = { 0 };
		char szDummyWord[1024] = { 0 };
		int actual_length = 0;
		receiveBuffer[0] = '\x02';
		actual_length++;
		int cnt = 0;
		callCnt++;
		if (callCnt % 2)
		{
			fileName = "c:\\temp\\scanVehicle\\tmp005964.txt";
		}
		fin = fopen(fileName.c_str(), "r");
		while (fgets(szLine, 1024, fin) != NULL)
		{
			char *ptr = strchr(szLine, '\n');;
			if (ptr != NULL)
			{
				*ptr = '\0';
			}

			ptr = strchr(szLine, ':');
			if (ptr != NULL)
			{
				if (1 == sscanf(ptr + 2, "%s", szDummyWord))
				{
					if (cnt > 0)
					{
						receiveBuffer[actual_length++] = ' ';
					}
					strcpy((char *)receiveBuffer + actual_length, szDummyWord);
					actual_length += strlen(szDummyWord);
				}
				cnt++;
			}
		}
		receiveBuffer[actual_length] = (char)'\x03';
		actual_length++;
		receiveBuffer[actual_length] = (char)'\x00';
		actual_length++;
		*pActual_length = actual_length;
		fclose(fin);
	}

	int SickScanRadar::parseDatagram(unsigned char *receiveBuffer, int actual_length, bool useBinaryProtocol)
	{
		int exitCode = ExitSuccess;

		simulateAsciiDatagram(receiveBuffer, &actual_length);

		// simulateAsciiDatagramFromFile(receiveBuffer, &actual_length);

		std::vector<SickScanRadarObject> objectList;
		std::vector<SickScanRadarRawTarget> rawTargetList;

		if (useBinaryProtocol)
		{
			throw std::logic_error("Binary protocol currently not supported.");
		}
		else
		{
			bool dataToProcess = false;
			char *buffer_pos = (char *)receiveBuffer;
			char *dstart = NULL;
			char *dend = NULL;
			int dlength = 0;
			dstart = strchr(buffer_pos, 0x02);
			if (dstart != NULL)
			{
				dend = strchr(dstart + 1, 0x03);
			}
			if ((dstart != NULL) && (dend != NULL))
			{
				dataToProcess = true; // continue parsing
				dlength = dend - dstart;
				*dend = '\0';
				dstart++;
				parseAsciiDatagram(dstart, dlength, objectList, rawTargetList);
			}
			else
			{
				dataToProcess = false;
			}

			sensor_msgs::PointCloud2 cloud_;

			int numTargets = rawTargetList.size();
			if (dataToProcess && (numTargets > 0))
			{
				std::string channelRawTargetId[] = { "x", "y", "z", "vrad","amplitude" };
				int numChannels = sizeof(channelRawTargetId) / sizeof(channelRawTargetId[0]);

				std::vector<float> valSingle;
				valSingle.resize(numChannels);
				cloud_.header.stamp = ros::Time::now();
				cloud_.header.frame_id = this->commonPtr->getConfigPtr()->frame_id;
				cloud_.header.seq = 0;
				cloud_.height = 1; // due to multi echo multiplied by num. of layers
				cloud_.width = numTargets;
				cloud_.is_bigendian = false;
				cloud_.is_dense = true;
				cloud_.point_step = numChannels * sizeof(float);
				cloud_.row_step = cloud_.point_step * cloud_.width;
				cloud_.fields.resize(numChannels);
				for (int i = 0; i < numChannels; i++) {
					cloud_.fields[i].name = channelRawTargetId[i];
					cloud_.fields[i].offset = i * sizeof(float);
					cloud_.fields[i].count = 1;
					cloud_.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
				}

				cloud_.data.resize(cloud_.row_step * cloud_.height);
				float *valPtr = (float *)(&(cloud_.data[0]));
				int off = 0;
				for (int i = 0; i < numTargets; i++)
				{
					float angle = deg2rad * rawTargetList[i].Azimuth();
					valSingle[0] = rawTargetList[i].Dist() * cos(angle);
					valSingle[1] = rawTargetList[i].Dist() * sin(angle);
					valSingle[2] = 0.0;
					valSingle[3] = rawTargetList[i].Vrad();
					valSingle[4] = rawTargetList[i].Ampl();

					for (int j = 0; j < numChannels; j++)
					{
						valPtr[off] = valSingle[j];
						off++;
					}
					}
#ifndef _MSC_VER
				cloud_pub_.publish(cloud_);
#else
				printf("PUBLISH:\n");
#endif
				}

			}
		return(exitCode);
		}

	}