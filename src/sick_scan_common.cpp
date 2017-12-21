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
*  Last modified: 21st Dec 2017
*
*      Authors:
*              Michael Lehning <michael.lehning@lehning.de>
*         Jochen Sprickerhof <jochen@sprickerhof.de>
*         Martin Günther <mguenthe@uos.de>
*
* Based on the TiM communication example by SICK AG.
*
*/

#ifdef _MSC_VER // compiling simulation for MS-Visual C++ - not defined for linux system
#pragma warning(disable: 4996)
#pragma warning(disable: 4267) // due to conversion warning size_t in the ros header file
#define _WIN32_WINNT 0x0501

#endif

#include <sick_scan/sick_scan_common.h>

#ifdef _MSC_VER
#include "sick_scan/rosconsole_simu.hpp"
#endif

#include <cstdio>
#include <cstring>

#define _USE_MATH_DEFINES
#include <math.h>

#ifndef rad2deg
#define rad2deg(x) ((x) / M_PI * 180.0)
#endif

#include <map>

static int getDiagnosticErrorCode() // workaround due to compiling error under Visual C++
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
	std::string stripControl(std::string s)
	{
		std::string dest;
		for (int i = 0; i < s.length(); i++)
		{

			if (s[i] >= ' ')
			{
				dest += s[i];
			}
			else
			{
				switch (s[i])
				{
				case 0x02: dest += "<STX>"; break;
				case 0x03: dest += "<ETX>"; break;
				}
			}
		}
		return(dest);
	}

	SickScanCommon::SickScanCommon(SickGenericParser* parser) :
		diagnosticPub_(NULL), expectedFrequency_(15.0), parser_(parser)
		// FIXME All Tims have 15Hz
	{
		init_cmdTables();
#ifndef _MSC_VER
		dynamic_reconfigure::Server<sick_scan::SickScanConfig>::CallbackType f;
		f = boost::bind(&sick_scan::SickScanCommon::update_config, this, _1, _2);
		dynamic_reconfigure_server_.setCallback(f);
#else
		// For simulation under MS Visual c++ the update config is switched off
		{
			SickScanConfig cfg;
			ros::NodeHandle tmp("~");
			double min_angle, max_angle, res_angle;
			tmp.getParam("min_ang", min_angle);
			tmp.getParam("max_ang", max_angle);
			tmp.getParam("res_ang", res_angle);
			cfg.min_ang = min_angle;
			cfg.max_ang = max_angle;
			cfg.skip = 0;
			update_config(cfg);
		}
#endif
		// datagram publisher (only for debug)
		ros::NodeHandle pn("~");
		pn.param<bool>("publish_datagram", publish_datagram_, false);
		if (publish_datagram_)
			datagram_pub_ = nh_.advertise<std_msgs::String>("datagram", 1000);


		// Pointcloud2 publisher
		//
		cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud", 100);

		// scan publisher
		pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 1000);

#ifndef _MSC_VER
		diagnostics_.setHardwareID("none");   // set from device after connection
		diagnosticPub_ = new diagnostic_updater::DiagnosedPublisher<sensor_msgs::LaserScan>(pub_, diagnostics_,
			// frequency should be target +- 10%.
			diagnostic_updater::FrequencyStatusParam(&expectedFrequency_, &expectedFrequency_, 0.1, 10),
			// timestamp delta can be from 0.0 to 1.3x what it ideally is.
			diagnostic_updater::TimeStampStatusParam(-1, 1.3 * 1.0 / expectedFrequency_ - config_.time_offset));
		ROS_ASSERT(diagnosticPub_ != NULL);
#endif
	}

	int SickScanCommon::stop_scanner()
	{
		/*
		 * Stop streaming measurements
		 */
		const char requestScanData0[] = { "\x02sEN LMDscandata 0\x03\0" };
		int result = sendSOPASCommand(requestScanData0, NULL);
		if (result != 0)
			// use printf because we cannot use ROS_ERROR from the destructor
			printf("\nSOPAS - Error stopping streaming scan data!\n");
		else
			printf("\nSOPAS - Stopped streaming scan data.\n");

		return result;
	}

	bool SickScanCommon::rebootScanner()
	{
		/*
		 * Set Maintenance access mode to allow reboot to be sent
		 */
		std::vector<unsigned char> access_reply;
		// changed from "03" to "3"
		int result = sendSOPASCommand("\x02sMN SetAccessMode 3 F4724744\x03\0", &access_reply);
		if (result != 0)
		{
			ROS_ERROR("SOPAS - Error setting access mode");
			diagnostics_.broadcast(getDiagnosticErrorCode(), "SOPAS - Error setting access mode.");
			return false;
		}
		std::string access_reply_str = replyToString(access_reply);
		if (access_reply_str != "sAN SetAccessMode 1")
		{
			ROS_ERROR_STREAM("SOPAS - Error setting access mode, unexpected response : " << access_reply_str);
			diagnostics_.broadcast(getDiagnosticErrorCode(), "SOPAS - Error setting access mode.");
			return false;
		}

		/*
		 * Send reboot command
		 */
		std::vector<unsigned char> reboot_reply;
		result = sendSOPASCommand("\x02sMN mSCreboot\x03\0", &reboot_reply);
		if (result != 0)
		{
			ROS_ERROR("SOPAS - Error rebooting scanner");
			diagnostics_.broadcast(getDiagnosticErrorCode(), "SOPAS - Error rebooting device.");
			return false;
		}
		std::string reboot_reply_str = replyToString(reboot_reply);
		if (reboot_reply_str != "sAN mSCreboot")
		{
			ROS_ERROR_STREAM("SOPAS - Error rebooting scanner, unexpected response : " << reboot_reply_str);
			diagnostics_.broadcast(getDiagnosticErrorCode(), "SOPAS - Error setting access mode.");
			return false;
		}

		ROS_INFO("SOPAS - Rebooted scanner");

		// Wait a few seconds after rebooting
		ros::Duration(15.0).sleep();

		return true;
	}

	SickScanCommon::~SickScanCommon()
	{
		delete diagnosticPub_;

		printf("sick_scan driver exiting.\n");
	}


	int SickScanCommon::setIpAddress(std::string ipAddress)
	{
		int result = -1;
		return result;

	}

	std::string SickScanCommon::generateExpectedAnswerString(const std::string requestStr)
	{
		std::string expectedAnswer = "";
		int i = 0;
		char cntWhiteCharacter = 0;
		int initalTokenCnt = 2; // number of initial token to identify command
		std::map<std::string, int> specialTokenLen;
		char firstToken[1024] = { 0 };
		specialTokenLen["sRI"] = 1; // for SRi-Command only the first token identify the command
		if (sscanf(requestStr.c_str(), "\x2%s", firstToken) == 1)
		{
			if (specialTokenLen.find(firstToken) != specialTokenLen.end())
			{
				initalTokenCnt = specialTokenLen[firstToken];

			}
		}


		for (int i = 0; i < requestStr.size(); i++)
		{
			if ((requestStr[i] == ' ') || (requestStr[i] == '\x3'))
			{
				cntWhiteCharacter++;
			}
			if (cntWhiteCharacter >= initalTokenCnt)
			{
				break;
			}
			if (requestStr[i] == '\x2')
			{
			}
			else
			{
				expectedAnswer += requestStr[i];
			}
		}


		std::map<std::string, std::string> keyWordMap;
		keyWordMap["sWN"] = "sWA";
		keyWordMap["sRN"] = "sRA";
		keyWordMap["sRI"] = "sRA";
		keyWordMap["sMN"] = "sAN";
		keyWordMap["sEN"] = "sEA";

		for (std::map<std::string, std::string>::iterator it = keyWordMap.begin(); it != keyWordMap.end(); it++)
		{

			std::string keyWord = it->first;
			std::string newKeyWord = it->second;

			size_t pos = expectedAnswer.find(keyWord);
			if (pos == std::string::npos)
			{

			}
			else
			{
				if (pos == 0)  // must be 0, if keyword has been found
				{
					expectedAnswer.replace(pos, keyWord.length(), newKeyWord);
				}
				else
				{
					ROS_WARN("Unexpected position of key identifier.\n");
				}
			}

		}
		return(expectedAnswer);
	}


	int SickScanCommon::sendSopasAndCheckAnswer(std::string requestStr, std::vector<unsigned char> *reply, int cmdId /*= -1*/)
	{
		int result = -1;

		std::string errString;
		if (cmdId == -1)
		{
			errString = "Error unexpected Sopas Answer for request " + requestStr;
		}
		else
		{
			errString = this->sopasCmdErrMsg[cmdId];
		}

		std::string expectedAnswer = generateExpectedAnswerString(requestStr);

		// send sopas cmd

		ROS_INFO("Sending  : %s", stripControl(requestStr).c_str());
		result = sendSOPASCommand(requestStr.c_str(), reply);
		std::string replyStr = replyToString(*reply);
		ROS_INFO("Receiving: %s", stripControl(replyStr).c_str());

		if (result != 0)
		{
			std::string tmpStr = "SOPAS Communication -" + errString;
			ROS_ERROR("%s\n", tmpStr.c_str());
			diagnostics_.broadcast(getDiagnosticErrorCode(), tmpStr);
		}
		else
		{
			std::string answerStr = replyToString(*reply);
			std::string searchPattern = generateExpectedAnswerString(requestStr);

			if (answerStr.find(searchPattern) != std::string::npos)
			{
				result = 0;
			}
			else
			{
				std::string tmpMsg = "Error Sopas answer mismatch " + errString + "Answer= >>>" + answerStr + "<<<";
				ROS_ERROR("%s\n", tmpMsg.c_str());
				diagnostics_.broadcast(getDiagnosticErrorCode(), tmpMsg);
				result = -1;
			}
		}
		return result;

	}
	//************************************
	// Method:    setAligmentMode
	// FullName:  sick_scan::SickScanCommon::setAligmentMode
	// Access:    public 
	// Returns:   int
	// Qualifier:
	// Parameter: int _AligmentMode
	//************************************
	int SickScanCommon::setAligmentMode(int _AligmentMode)
	{
		const int MAX_STR_LEN = 1024;
		int result = -1;

		const char *setAligmentModeMask = sopasCmdMaskVec[CMD_ALIGNMENT_MODE].c_str(); // "sWN MMAlignmentMode %d"
		char setAligmentMode[MAX_STR_LEN];
		std::vector<unsigned char> outputAligmentMode;
		sprintf(setAligmentMode, setAligmentModeMask, _AligmentMode);
		result = sendSopasAndCheckAnswer(setAligmentMode, &outputAligmentMode, CMD_ALIGNMENT_MODE);
		return result;
	}

	int SickScanCommon::setMeanFilter(bool _active, int _numberOfScans)
	{
		const int MAX_STR_LEN = 1024;
		int result = -1;
		int intactive = _active ? 1 : 0;
		// const char setMeanFilterMask[] = "\x02sWN LFPmeanfilter %d +%d 1\x03";

		char setMeanFilter[MAX_STR_LEN];
		std::vector<unsigned char> outputMeanFilter;
		sprintf(setMeanFilter, sopasCmdMaskVec[CMD_SET_MEAN_FILTER].c_str(), intactive, _numberOfScans);
		result = sendSopasAndCheckAnswer(setMeanFilter, &outputMeanFilter, CMD_SET_MEAN_FILTER);
		return result;
	}

	int SickScanCommon::setApplicationMode(bool _active, int _mode)
		//0=RANG (Ranging) 1=FEVL (Field Application)
	{
		const int MAX_STR_LEN = 1024;
		int result = -1;
		std::string modeNametmp = _mode ? "FEVL" : "RANG";
		int intactive = _active ? 1 : 0;
		//char *modeName = const_cast<char*>(modeNametmp.c_str());
		//const char setModeMask[] = "\x02sWN SetActiveApplications 1 %s %d\x03";
		char setAplicationMode[MAX_STR_LEN];
		std::vector<unsigned char> outputMode;
		sprintf(setAplicationMode, sopasCmdMaskVec[CMD_APLICATION_MODE].c_str(), modeNametmp, intactive);
		result = sendSopasAndCheckAnswer(setAplicationMode, &outputMode, CMD_APLICATION_MODE);
		return result;
	}


	int SickScanCommon::ActivateStandBy(void)
	{
		const int MAX_STR_LEN = 1024;
		int result = -1;
		std::vector<unsigned char> outputsetStandby;
		result = sendSOPASCommand(sopasCmdVec[CMD_ACTIVATE_STANDBY].c_str(), &outputsetStandby);
		return result;
	}

	int SickScanCommon::setParticleFilter(bool _active, int _particleThreshold)
		//actually only 500 mm as trashold is allowed;0 as trashold deactivates filter
	{
		const int MAX_STR_LEN = 1024;
		int result = -1;
		int intactive = _active ? 1 : 0;
		char setParticleFilter[MAX_STR_LEN];
		std::vector<unsigned char> outputMode;
		sprintf(setParticleFilter, sopasCmdMaskVec[CMD_SET_PARTICLE_FILTER].c_str(), intactive, _particleThreshold);
		result = sendSopasAndCheckAnswer(setParticleFilter, &outputMode, CMD_SET_PARTICLE_FILTER);
		return result;
	}


	//actually only 500 mm as trashold is allowed;0 as trashold deactivates filter


	bool SickScanCommon::testSettingIpAddress() {
		bool result = false;
		return result;
	}

	bool SickScanCommon::testsetParticleFilter()
	{
		bool result = false;
		if (0 == setParticleFilter(true, 500))
		{
			result = true;
		}
		setParticleFilter(false, 500);
		return result;
	}

	bool SickScanCommon::testsetMeanFilter()
	{
		bool result = false;
		if (0 == setMeanFilter(true, 10))
		{
			result = true;
		}
		setMeanFilter(false, 10);
		return result;
	}

	bool SickScanCommon::testsetAligmentMode()
	{
		bool result = false;
		if (0 == setAligmentMode(0))
		{
			result = true;
		}
		return result;
	}

	bool SickScanCommon::testsetActivateStandBy()
	{
		bool result = false;
		if (0 == ActivateStandBy())
		{
			result = true;
		}
		return result;
	}

	bool SickScanCommon::testsetApplicationMode()
	{
		bool result = false;
		int commandresult;
		commandresult = setApplicationMode(1, 0);
		if (commandresult == 0)
		{
			result = true;
		}
		return result;
	}



	//************************************
	// Method:    init
	// FullName:  sick_scan::SickScanCommon::init
	// Access:    virtual public 
	// Returns:   int
	// Qualifier:
	//************************************
	int SickScanCommon::init()
	{
		int result = init_device();
		if (result != 0)
		{
			ROS_FATAL("Failed to init device: %d", result);
			return result;
		}
		result = init_scanner();
		if (result != 0)
		{
			ROS_FATAL("Failed to init scanner: %d", result);
		}
		return result;
	}

	int SickScanCommon::init_cmdTables()
	{
		sopasCmdVec.resize(SickScanCommon::CMD_END);
		sopasCmdMaskVec.resize(SickScanCommon::CMD_END);  // you for cmd with variable content. sprintf should print into corresponding sopasCmdVec
		sopasCmdErrMsg.resize(SickScanCommon::CMD_END);  // you for cmd with variable content. sprintf should print into corresponding sopasCmdVec
		sopasReplyVec.resize(SickScanCommon::CMD_END);
		sopasReplyStrVec.resize(SickScanCommon::CMD_END);

		std::string unknownStr = "Command or Error message not defined";
		for (int i = 0; i < SickScanCommon::CMD_END; i++)
		{
			sopasCmdVec[i] = unknownStr;
			sopasCmdMaskVec[i] = unknownStr;  // you for cmd with variable content. sprintf should print into corresponding sopasCmdVec
			sopasCmdErrMsg[i] = unknownStr;  // you for cmd with variable content. sprintf should print into corresponding sopasCmdVec
			sopasReplyVec[i] = unknownStr;
			sopasReplyStrVec[i] = unknownStr;
		}

		sopasCmdVec[CMD_DEVICE_IDENT] = "\x02sRI 0\x03\0";
		sopasCmdVec[CMD_SERIAL_NUMBER] = "\x02sRN SerialNumber\x03\0";
		sopasCmdVec[CMD_FIRMWARE_VERSION] = "\x02sRN FirmwareVersion\x03\0";
		sopasCmdVec[CMD_DEVICE_STATE] = "\x02sRN SCdevicestate\x03\0";
		sopasCmdVec[CMD_OPERATION_HOURS] = "\x02sRN ODoprh\x03\0";
		sopasCmdVec[CMD_POWER_ON_COUNT] = "\x02sRN ODpwrc\x03\0";
		sopasCmdVec[CMD_LOCATION_NAME] = "\x02sRN LocationName\x03\0";
		sopasCmdVec[CMD_ACTIVATE_STANDBY] = "\x02sMN LMCstandby\x03";
		sopasCmdVec[CMD_SET_ACCESS_MODE_3] = "\x02sMN SetAccessMode 3 F4724744\x03\0";
		sopasCmdVec[CMD_GET_OUTPUT_RANGES] = "\x02sRN LMPoutputRange\x03";
		sopasCmdVec[CMD_RUN] = "\x02sMN Run\x03\0";
		sopasCmdVec[CMD_STOP_SCANDATA] = "\x02sEN LMDscandata 0\x03";
		sopasCmdVec[CMD_START_SCANDATA] = "\x02sEN LMDscandata 1\x03";
		sopasCmdVec[CMD_START_MEASUREMENT] = "\x02sMN LMCstartmeas\x03";
		sopasCmdVec[CMD_STOP_MEASUREMENT] = "\x02sMN LMCstopmeas\x03";

		// defining cmd mask for cmds with variable input
		sopasCmdMaskVec[CMD_SET_PARTICLE_FILTER] = "\x02sWN LFPparticle %d %d\x03";
		sopasCmdMaskVec[CMD_SET_MEAN_FILTER] = "\x02sWN LFPmeanfilter %d +%d 1\x03";
		sopasCmdMaskVec[CMD_ALIGNMENT_MODE] = "\x02sWN MMAlignmentMode %d\x03";
		sopasCmdMaskVec[CMD_APLICATION_MODE] = "\x02sWN SetActiveApplications 1 %s %d\x03";
		sopasCmdMaskVec[CMD_SET_OUTPUT_RANGES] = "\x02sWN LMPoutputRange 1 %X %X %X\x03";
		sopasCmdMaskVec[CMD_SET_PARTIAL_SCANDATA_CFG] = "\x02sWN LMDscandatacfg %02d 00 %d 0 0 00 00 0 0 0 0 1\x03";
		sopasCmdMaskVec[CMD_SET_ECHO_FILTER] = "\x02sWN FREchoFilter %d\x03";

		//error Messages
		sopasCmdErrMsg[CMD_DEVICE_IDENT] = "Error reading device ident";
		sopasCmdErrMsg[CMD_SERIAL_NUMBER] = "Error reading SerialNumber";
		sopasCmdErrMsg[CMD_FIRMWARE_VERSION] = "Error reading FirmwareVersion";
		sopasCmdErrMsg[CMD_DEVICE_STATE] = "Error reading SCdevicestate";
		sopasCmdErrMsg[CMD_OPERATION_HOURS] = "Error reading operation hours";
		sopasCmdErrMsg[CMD_POWER_ON_COUNT] = "Error reading operation power on counter";
		sopasCmdErrMsg[CMD_LOCATION_NAME] = "Error reading Locationname";
		sopasCmdErrMsg[CMD_ACTIVATE_STANDBY] = "Error acticvating Standby";
		sopasCmdErrMsg[CMD_SET_PARTICLE_FILTER] = "Error setting Particelefilter";
		sopasCmdErrMsg[CMD_SET_MEAN_FILTER] = "Error setting Meanfilter";
		sopasCmdErrMsg[CMD_ALIGNMENT_MODE] = "Error setting Alignmentmode";
		sopasCmdErrMsg[CMD_APLICATION_MODE] = "Error setting Meanfilter";
		sopasCmdErrMsg[CMD_SET_ACCESS_MODE_3] = "Error Acces Mode";
		sopasCmdErrMsg[CMD_SET_OUTPUT_RANGES] = "Error setting angular ranges";
		sopasCmdErrMsg[CMD_GET_OUTPUT_RANGES] = "Error reading angle range";
		sopasCmdErrMsg[CMD_RUN] = "FATAL ERROR unable to start RUN mode!";
		sopasCmdErrMsg[CMD_SET_PARTIAL_SCANDATA_CFG] = "Error setting Scandataconfig";
		sopasCmdErrMsg[CMD_STOP_SCANDATA] = "Error stopping scandata output";
		sopasCmdErrMsg[CMD_START_SCANDATA] = "Error starting Scandata output";

		// ML: Add hier more useful cmd and mask entries

		// After definition of command, we specify the command sequence for scanner initalisation

		// try for MRS1104

		sopasCmdChain.push_back(CMD_SET_ACCESS_MODE_3);

		if (parser_->getCurrentParamPtr()->getNumberOfLayers() == 1)
		{
		     // do not stop measurement for TiM571 otherweise the scanner would not start after start measurement	
		}
		else
		{
			sopasCmdChain.push_back(CMD_STOP_MEASUREMENT);
		}
		sopasCmdChain.push_back(CMD_DEVICE_IDENT);
		sopasCmdChain.push_back(CMD_SERIAL_NUMBER);
		sopasCmdChain.push_back(CMD_FIRMWARE_VERSION);
		sopasCmdChain.push_back(CMD_DEVICE_STATE);
		sopasCmdChain.push_back(CMD_OPERATION_HOURS);
		sopasCmdChain.push_back(CMD_POWER_ON_COUNT);
		sopasCmdChain.push_back(CMD_LOCATION_NAME);

		// ML: define here the command sequence

		return(0);

	}

	int SickScanCommon::init_scanner()
	{

		const int MAX_STR_LEN = 1024;

		int maxNumberOfEchos = 1;



		maxNumberOfEchos = this->parser_->getCurrentParamPtr()->getNumberOfMaximumEchos();  // 1 for TIM 571, 3 for MRS1104, 5 for 6000

		bool rssiFlag = false;
		int activeEchos = 0;
		ros::NodeHandle pn("~");
		pn.getParam("intensity", rssiFlag);

		// parse active_echos entry and set flag array
		pn.getParam("active_echos", activeEchos);

		ROS_INFO("Parameter setting for <active_echo: %d>", activeEchos);
		std::vector<bool> outputChannelFlag;
		outputChannelFlag.resize(maxNumberOfEchos);
		int i;
		int numOfFlags = 0;
		for (i = 0; i < outputChannelFlag.size(); i++)
		{
			if (activeEchos & (1 << i))
			{
				outputChannelFlag[i] = true;
				numOfFlags++;
			}
			else
			{
				outputChannelFlag[i] = false;
			}
		}

		if (numOfFlags == 0) // Fallback-Solution
		{
			outputChannelFlag[0] = true;
			numOfFlags = 1;
			ROS_WARN("Activate at least one echo.");
		}

		int result;
		//================== DEFINE ANGULAR SETTING ==========================
		int    angleRes10000th = 0;
		int    angleStart10000th = 0;
		int    angleEnd10000th = 0;

		// Mainloop for initial SOPAS cmd-Handling
		//
		// To add new commands do the followings:
		// 1. Define new enum-entry in the enumumation "enum SOPAS_CMD" in the file sick_scan_common.h
		// 2. Define new command sequence in the member function init_cmdTables()
		// 3. Add cmd-id in the sopasCmdChain to trigger sending of command.
		// 4. Add handling of reply by using for the pattern "REPLY_HANDLING" in this code and adding a new case instruction.
		// That's all!

		for (int i = 0; i < this->sopasCmdChain.size(); i++)
		{
			int cmdId = sopasCmdChain[i]; // get next command
			std::string sopasCmd = sopasCmdVec[cmdId];
			std::vector<unsigned char> replyDummy;
			result = sendSopasAndCheckAnswer(sopasCmd.c_str(), &replyDummy);
			ROS_DEBUG("Command: %s", stripControl(sopasCmd).c_str());
			if (result != 0)
			{
				ROS_ERROR("%s", sopasCmdErrMsg[cmdId].c_str());
				diagnostics_.broadcast(getDiagnosticErrorCode(), sopasCmdErrMsg[cmdId]);
			}
			else
			{
				sopasReplyStrVec[cmdId] = replyToString(replyDummy);
			}

			//============================================
			// Handle reply of specific commands here by inserting new case instruction
			// REPLY_HANDLING
			//============================================
			switch (cmdId)
			{
				/*
				SERIAL_NUMBER: Device ident must be read before!
				*/

			case CMD_SERIAL_NUMBER:
				diagnostics_.setHardwareID(sopasReplyStrVec[CMD_DEVICE_IDENT] + " " + sopasReplyStrVec[CMD_SERIAL_NUMBER]);

				if (!isCompatibleDevice(sopasReplyStrVec[CMD_DEVICE_IDENT]))
					return ExitFatal;

				break;
				/*
				DEVICE_STATE
				*/
			case CMD_DEVICE_STATE:
			{
				int deviceState = -1;
				/*
				* Process device state, 0=Busy, 1=Ready, 2=Error
				* If configuration parameter is set, try resetting device in error state
				*/

				if (1 == sscanf(sopasReplyStrVec[CMD_DEVICE_STATE].c_str(), "sRA SCdevicestate %d", &deviceState))
				{
					switch (deviceState)
					{
					case 0: 					ROS_DEBUG("Laser is busy"); break;
					case 1:						ROS_DEBUG("Laser is ready"); break;
					case 2:	ROS_ERROR_STREAM("Laser reports error state : " << sopasReplyStrVec[CMD_DEVICE_STATE]);
						if (config_.auto_reboot)
						{
							rebootScanner();
						};
						break;
					default:	ROS_WARN_STREAM("Laser reports unknown devicestate : " << sopasReplyStrVec[CMD_DEVICE_STATE]);
						break;
					}
				}
			}
			break;

			case CMD_OPERATION_HOURS:
			{
				int operationHours = -1;
				/*
				* Process device state, 0=Busy, 1=Ready, 2=Error
				* If configuration parameter is set, try resetting device in error state
				*/

				if (1 == sscanf(sopasReplyStrVec[CMD_OPERATION_HOURS].c_str(), "sRA ODoprh %x", &operationHours))
				{
					double hours = 0.1 * operationHours;
					pn.setParam("operationHours", hours);
				}
			}
			break;

			case CMD_POWER_ON_COUNT:
			{
				int powerOnCount = -1;
				if (1 == sscanf(sopasReplyStrVec[CMD_POWER_ON_COUNT].c_str(), "sRA ODpwrc %x", &powerOnCount))
				{
					pn.setParam("powerOnCount", powerOnCount);
				}
			}
			break;

			case CMD_LOCATION_NAME:
			{
				char szLocationName[MAX_STR_LEN] = { 0 };
				const char *strPtr = sopasReplyStrVec[CMD_LOCATION_NAME].c_str();
				const char *searchPattern = "sRA LocationName D";
				strcpy(szLocationName, "unknown location");

				if (strstr(strPtr, searchPattern) == strPtr) {
					const char *ptr = strPtr + strlen(searchPattern);
					strcpy(szLocationName, ptr);

				}
				else {
					ROS_WARN("Location command not supported.\n");
				}
				pn.setParam("locationName", std::string(szLocationName));
			}
			break;
			case CMD_GET_PARTIAL_SCANDATA_CFG:
			{

				const char *strPtr = sopasReplyStrVec[CMD_LOCATION_NAME].c_str();
				ROS_INFO("Config: %s\n", strPtr);
			}
			break;
			// ML: add here reply handling
			}
		}



		//-----------------------------------------------------------------
		//
		// Set Access Mode to change parameter afterward
		//
		//-----------------------------------------------------------------
		// sMN SetAccessMode 04 81BE23AA (vgl. https://www.google.de/url?sa=t&rct=j&q=&esrc=s&source=web&cd=2&cad=rja&uact=8&ved=0ahUKEwjTqbS_3NnXAhWCSBQKHfOwAZkQFgguMAE&url=https%3A%2F%2Fwww.sick.com%2Fmedia%2Fpdf%2F7%2F17%2F617%2FIM0056617.PDF&usg=AOvVaw16Wc_EqJALJttj1WjNnWY5 )


		//-----------------------------------------------------------------
		//
		// Try to read angular resolution of specific scanner.
		// This is recommended to decide between TiM551 and TiM561/TiM571 capabilities
		// The TiM551 has an angular resolution of 1.000 [deg]
		// The TiM561 and TiM571 have an angular resolution of 0.333 [deg]
		//-----------------------------------------------------------------

		angleRes10000th = (int)(boost::math::round(10000.0   * this->parser_->getCurrentParamPtr()->getAngularDegreeResolution()));
		std::vector<unsigned char> askOutputAngularRangeReply;
		result = sendSopasAndCheckAnswer(sopasCmdVec[CMD_GET_OUTPUT_RANGES].c_str(), &askOutputAngularRangeReply);
		if (0 == result)
		{
			int askTmpAngleRes10000th, askTmpAngleStart10000th, askTmpAngleEnd10000th;
			char dummy0[MAX_STR_LEN] = { 0 };
			char dummy1[MAX_STR_LEN] = { 0 };
			int  dummyInt = 0;
			std::string askOutputAngularRangeStr = replyToString(askOutputAngularRangeReply);
			int numArgs = sscanf(askOutputAngularRangeStr.c_str(), "%s %s %d %X %X %X", dummy0, dummy1,
				&dummyInt,
				&askTmpAngleRes10000th,
				&askTmpAngleStart10000th,
				&askTmpAngleEnd10000th);
			if (6 == numArgs)
			{
				double askTmpAngleRes = askTmpAngleRes10000th / 10000.0;
				double askTmpAngleStart = askTmpAngleStart10000th / 10000.0;
				double askTmpAngleEnd = askTmpAngleEnd10000th / 10000.0;

				angleRes10000th = askTmpAngleRes10000th;
				ROS_INFO("Angle resolution of scanner is %0.5lf [deg]  (in 1/10000th deg: 0x%X)", askTmpAngleRes, askTmpAngleRes10000th);

			}
		}
		//-----------------------------------------------------------------
		//
		// Set Min- und Max scanning angle given by config
		//
		//-----------------------------------------------------------------

		ROS_INFO("MIN_ANG: %8.3f [rad] %8.3f [deg]", config_.min_ang, rad2deg(this->config_.min_ang));
		ROS_INFO("MAX_ANG: %8.3f [rad] %8.3f [deg]", config_.max_ang, rad2deg(this->config_.max_ang));

		// convert to 10000th degree
		double minAngSopas = rad2deg(this->config_.min_ang) + 90;
		double maxAngSopas = rad2deg(this->config_.max_ang) + 90;
		angleStart10000th = (int)(boost::math::round(10000.0 * minAngSopas));
		angleEnd10000th = (int)(boost::math::round(10000.0   * maxAngSopas));

		char requestOutputAngularRange[MAX_STR_LEN];
		// special for LMS1000
		if (this->parser_->getCurrentParamPtr()->getScannerName().compare(SICK_SCANNER_LMS_1XXX_NAME) == 0)
		{
			ROS_WARN("Angular settings for LMS 1000 not reliable.\n");
			double askAngleStart = -137.0;
			double askAngleEnd = +137.0;

			this->config_.min_ang = askAngleStart / 180.0 * M_PI;
			this->config_.max_ang = askAngleEnd / 180.0 * M_PI;
		}
		else
		{
			std::vector<unsigned char> outputAngularRangeReply;
			sprintf(requestOutputAngularRange, sopasCmdMaskVec[CMD_SET_OUTPUT_RANGES].c_str(), angleRes10000th, angleStart10000th, angleEnd10000th);
			result = sendSopasAndCheckAnswer(requestOutputAngularRange, &outputAngularRangeReply);
		}

		//-----------------------------------------------------------------
		//
		// Get Min- und Max scanning angle from the scanner to verify angle setting and correct the config, if something went wrong.
		//
		// IMPORTANT:
		// Axis Orientation in ROS differs from SICK AXIS ORIENTATION!!!
		// In relation to a body the standard is:
		// x forward
		// y left
		// z up
		// see http://www.ros.org/reps/rep-0103.html#coordinate-frame-conventions for more details
		//-----------------------------------------------------------------

		askOutputAngularRangeReply.clear();
		result = sendSopasAndCheckAnswer(sopasCmdVec[CMD_GET_OUTPUT_RANGES].c_str(), &askOutputAngularRangeReply);
		if (result == 0)
		{
			int askAngleRes10000th, askAngleStart10000th, askAngleEnd10000th;
			char dummy0[MAX_STR_LEN] = { 0 };
			char dummy1[MAX_STR_LEN] = { 0 };
			int  dummyInt = 0;
			std::string askOutputAngularRangeStr = replyToString(askOutputAngularRangeReply);
			sscanf(askOutputAngularRangeStr.c_str(), "%s %s %d %X %X %X", dummy0, dummy1,
				&dummyInt,
				&askAngleRes10000th,
				&askAngleStart10000th,
				&askAngleEnd10000th);
			double askAngleRes = askAngleRes10000th / 10000.0;
			double askAngleStart = askAngleStart10000th / 10000.0;
			double askAngleEnd = askAngleEnd10000th / 10000.0;

			askAngleStart -= 90; // angle in ROS relative to y-axis
			askAngleEnd -= 90; // angle in ROS relative to y-axis
			this->config_.min_ang = askAngleStart / 180.0 * M_PI;
			this->config_.max_ang = askAngleEnd / 180.0 * M_PI;
			ros::NodeHandle nhPriv("~");
			nhPriv.setParam("min_ang", this->config_.min_ang); // update parameter setting with "true" values read from scanner
			nhPriv.setParam("max_ang", this->config_.max_ang); // update parameter setting with "true" values read from scanner
			ROS_INFO("MIN_ANG (after command verification): %8.3f [rad] %8.3f [deg]", config_.min_ang, rad2deg(this->config_.min_ang));
			ROS_INFO("MAX_ANG (after command verification): %8.3f [rad] %8.3f [deg]", config_.max_ang, rad2deg(this->config_.max_ang));
		}



		//-----------------------------------------------------------------
		//
		// Configure the data content of scan messing regarding to config.
		//
		//-----------------------------------------------------------------
		/*
		see 4.3.1 Configure the data content for the scan in the
		*/
		//                              1    2     3
		// Prepare flag array for echos

		outputChannelFlagId = 0x00;
		for (int i = 0; i < outputChannelFlag.size(); i++)
		{
			outputChannelFlagId |= ((outputChannelFlag[i] == true) << i);
		}
		outputChannelFlagId = std::max(outputChannelFlagId, 1); // at least one channel must be set



		// set scanning angle for tim5xx and for mrs1104
		if ((this->parser_->getCurrentParamPtr()->getNumberOfLayers() == 1)
			|| (this->parser_->getCurrentParamPtr()->getNumberOfLayers() == 4)
			)
		{
			char requestLMDscandatacfg[MAX_STR_LEN];
			if (this->parser_->getCurrentParamPtr()->getScannerName().compare(SICK_SCANNER_LMS_1XXX_NAME) == 0)
			{
				ROS_WARN("ECHO config for LMS 1000 not reliable\n");
			}
			else
			{
				// Uses sprintf-Mask to set bitencoded echos and rssi enable flag
				sprintf(requestLMDscandatacfg, sopasCmdMaskVec[CMD_SET_PARTIAL_SCANDATA_CFG].c_str(), outputChannelFlagId, rssiFlag ? 1 : 0);

				std::vector<unsigned char> lmdScanDataCfgReply;
				result = sendSopasAndCheckAnswer(requestLMDscandatacfg, &lmdScanDataCfgReply);
			}
		}

		// CONFIG ECHO-Filter (only for MRS1000 not available for TiM5xx
		if (this->parser_->getCurrentParamPtr()->getNumberOfLayers() >= 4)
		{
			char requestEchoSetting[MAX_STR_LEN];
			int filterEchoSetting = 0;
			pn.getParam("filter_echos", filterEchoSetting); // filter_echos
			if (filterEchoSetting < 0) filterEchoSetting = 0;
			if (filterEchoSetting > 2) filterEchoSetting = 2;
			// Uses sprintf-Mask to set bitencoded echos and rssi enable flag
			sprintf(requestEchoSetting, sopasCmdMaskVec[CMD_SET_ECHO_FILTER].c_str(), filterEchoSetting);
			std::vector<unsigned char> outputFilterEchoRangeReply;
			result = sendSopasAndCheckAnswer(requestEchoSetting, &outputFilterEchoRangeReply);


		}



		//-----------------------------------------------------------------
			//
			// Start sending LMD-Scandata messages
			//
			//-----------------------------------------------------------------
		std::vector<int> startProtocolSequence;


		// RUN and START MEASUREMENT must be called in different sequence for TiM5xx and MRS1104
		if (this->parser_->getCurrentParamPtr()->getNumberOfLayers() == 1)  // TiM5xx
		{
			startProtocolSequence.push_back(CMD_START_MEASUREMENT);
			startProtocolSequence.push_back(CMD_RUN);  // leave user level
		}
		else
		{
			startProtocolSequence.push_back(CMD_RUN);  // leave user level
			startProtocolSequence.push_back(CMD_START_MEASUREMENT);

		}
		startProtocolSequence.push_back(CMD_START_SCANDATA);


		std::vector<int>::iterator it;
		for (it = startProtocolSequence.begin(); it != startProtocolSequence.end(); it++)
		{
			int cmdId = *it;
			std::vector<unsigned char> tmpReply;
			sendSopasAndCheckAnswer(sopasCmdVec[cmdId].c_str(), &tmpReply);


			if (cmdId == CMD_START_MEASUREMENT)
			{

				if (this->parser_->getCurrentParamPtr()->getNumberOfLayers() == 1)
				{
					// do nothing for tim5xx
				}
				else
				{
				int maxWaitForDeviceStateReady = 45;   // max. 30 sec. (see manual)
				bool scannerReady = false;
				for (int i = 0; i < maxWaitForDeviceStateReady; i++)
				{
					double shortSleepTime = 1.0;
					std::string sopasCmd = sopasCmdVec[CMD_DEVICE_STATE];
					std::vector<unsigned char> replyDummy;
					int tmpResult = sendSopasAndCheckAnswer(sopasCmd.c_str(), &replyDummy);
					std::string reply = replyToString(replyDummy);
					ROS_INFO("STATE: %s\n", reply.c_str());
					int deviceState = 0;
					if (1 == sscanf(reply.c_str(), "sRA SCdevicestate %d", &deviceState))
					{
						if (deviceState == 1) // scanner is ready
						{
							scannerReady = true; // interrupt waiting for scanner ready
							ROS_INFO("Scanner ready for measurement after %d [sec]", i);
							break;
						}

					}
					ROS_INFO("Wait for scanner ready state for %d Secs",i);
					ros::Duration(shortSleepTime).sleep();

				}
				}
			}
			tmpReply.clear();

		}
		return ExitSuccess;
	}

	std::string sick_scan::SickScanCommon::replyToString(const std::vector<unsigned char> &reply)
	{
		std::string reply_str;
		for (std::vector<unsigned char>::const_iterator it = reply.begin(); it != reply.end(); it++)
		{
			if (*it > 13) // filter control characters for display
			{
				reply_str.push_back(*it);
			}
		}
		return reply_str;
	}

	//************************************
	// Method:    isCompatibleDevice
	// FullName:  sick_scan::SickScanCommon::isCompatibleDevice
	// Access:    protected 
	// Returns:   bool
	// Qualifier: const
	// Parameter: const std::string identStr
	//			  Must be checked for this implementation
	//************************************
	bool sick_scan::SickScanCommon::isCompatibleDevice(const std::string identStr) const
	{
		char device_string[7];
		int version_major = -1;
		int version_minor = -1;

		// special for TiM3-Firmware
		if (sscanf(identStr.c_str(), "sRA 0 6 %6s E V%d.%d", device_string,
			&version_major, &version_minor) == 3
			&& strncmp("TiM3", device_string, 4) == 0
			&& version_major >= 2 && version_minor >= 50)
		{
			ROS_ERROR("This scanner model/firmware combination does not support ranging output!");
			ROS_ERROR("Supported scanners: TiM5xx: all firmware versions; TiM3xx: firmware versions < V2.50.");
			ROS_ERROR("This is a %s, firmware version %d.%d", device_string, version_major, version_minor);

			return false;
		}

		if (sscanf(identStr.c_str(), "sRA 0 6 %6s E V%d.%d", device_string, &version_major, &version_minor) == 3)
		{
			std::string devStr = device_string;
			bool supported = false;

			if (devStr.compare(0, 4, "TiM5") == 0)
			{
				supported = true;
			}

			if (supported == true)
			{
				ROS_INFO("Device %s V%d.%d found and supported by this driver.", identStr.c_str(), version_major, version_minor);
			}
			else
			{
				ROS_WARN("Device %s V%d.%d found and maybe unsupported by this driver.", device_string, version_major, version_minor);
				ROS_WARN("Full SOPAS answer: %s", identStr.c_str());

			}
		}

		return true;
	}

	int SickScanCommon::loopOnce()
	{
		diagnostics_.update();

		unsigned char receiveBuffer[65536];
		int actual_length = 0;
		static unsigned int iteration_count = 0;

		int result = get_datagram(receiveBuffer, 65536, &actual_length);
		if (result != 0)
		{
			ROS_ERROR("Read Error when getting datagram: %i.", result);
			diagnostics_.broadcast(getDiagnosticErrorCode(), "Read Error when getting datagram.");
			return ExitError; // return failure to exit node
		}
		if (actual_length <= 0)
			return ExitSuccess; // return success to continue looping

		// ----- if requested, skip frames
		if (iteration_count++ % (config_.skip + 1) != 0)
			return ExitSuccess;

		if (publish_datagram_)
		{
			std_msgs::String datagram_msg;
			datagram_msg.data = std::string(reinterpret_cast<char*>(receiveBuffer));
			datagram_pub_.publish(datagram_msg);
		}

		sensor_msgs::LaserScan msg;

		/*
		 * datagrams are enclosed in <STX> (0x02), <ETX> (0x03) pairs
		 */
		char* buffer_pos = (char*)receiveBuffer;
		char *dstart, *dend;
		bool dumpDbg = false;
		while ((dstart = strchr(buffer_pos, 0x02)) && (dend = strchr(dstart + 1, 0x03)))
		{
			size_t dlength = dend - dstart;
			*dend = '\0';

			dstart++;

			if (dumpDbg)
			{
				{
					static int cnt = 0;
					char szFileName[255];

#ifdef _MSC_VER
					sprintf(szFileName, "c:\\temp\\dump%05d.txt", cnt);
#else
					sprintf(szFileName, "/tmp/dump%05d.txt", cnt);
#endif
					FILE *fout;
					fout = fopen(szFileName, "wb");
					fwrite(dstart, dlength, 1, fout);
					fclose(fout);
					cnt++;
				}
			}

			// HEADER of data followed by DIST1 ... DIST2 ... DIST3 .... RSSI1 ... RSSI2.... RSSI3...

			// <frameid>_<sign>00500_DIST[1|2|3]
			int numEchos = 1;
			int echoMask;
			// numEchos contains number of echos (1..5)
			// _msg holds ALL data of all echos
			int success = parser_->parse_datagram(dstart, dlength, config_, msg, numEchos, echoMask);
			bool publishPointCloud = true; // for MRS1000
			const int maxAllowedEchos = 5;

			int numValidEchos = 0;
			int aiValidEchoIdx[maxAllowedEchos] = { 0 };

			if (success == ExitSuccess)
			{
				std::vector<float> rangeTmp = msg.ranges;  // copy all range value
				std::vector<float> intensityTmp = msg.intensities; // copy all intensity value
					// XXX  - HIER MEHRERE SCANS publish, falls Mehrzielszenario läuft
				if (numEchos > 5)
				{
					ROS_WARN("Too much echos");
				}
				else
				{

					int startOffset = 0;
					int endOffset = 0;
					int echoPartNum = msg.ranges.size() / numEchos;
					for (int i = 0; i < numEchos; i++)
					{

						bool sendMsg = false;
						if ((echoMask & (1 << i)) & 	outputChannelFlagId)
						{
							aiValidEchoIdx[numValidEchos] = i; // save index
							numValidEchos++;
							sendMsg = true;
						}
						startOffset = i * echoPartNum;
						endOffset = (i + 1) * echoPartNum;

						msg.ranges.clear();
						msg.intensities.clear();
						msg.ranges = std::vector<float>(
							rangeTmp.begin() + startOffset,
							rangeTmp.begin() + endOffset);
						if (endOffset >= intensityTmp.size() && (intensityTmp.size() > 0))
						{
							msg.intensities = std::vector<float>(
								intensityTmp.begin() + startOffset,
								intensityTmp.begin() + endOffset);
						}
						else
						{
							msg.intensities.resize(echoPartNum); // fill with zeros
						}
						// msg.header.frame_id = ""
						{
							// numEchos
							char szTmp[255] = { 0 };
							if (this->parser_->getCurrentParamPtr()->getNumberOfLayers() > 1)
							{
								sprintf(szTmp, "%s_%+04d_DIST%d", config_.frame_id.c_str(), msg.header.seq, i + 1);
								msg.header.frame_id = std::string(szTmp);
							}
						}

#ifndef _MSC_VER
						if (sendMsg & 	outputChannelFlagId)  // publish only configured channels - workaround for cfg-bug MRS1104
						{
							diagnosticPub_->publish(msg);
						}
#else
						printf("MSG received...");
#endif
					}
				}


				if (publishPointCloud == true)
				{
					double elevationAngleDegree = 0.0;

					const int numChannels = 4; // x y z i (for intensity)
					int numOfLayers = parser_->getCurrentParamPtr()->getNumberOfLayers();


					cloud_.header.frame_id = config_.frame_id;
					cloud_.header.seq = 0;
					cloud_.height = numOfLayers * numValidEchos; // due to multi echo multiplied by num. of layers
					cloud_.width = msg.ranges.size();
					cloud_.is_bigendian = false;
					cloud_.is_dense = true;
					cloud_.point_step = numChannels * sizeof(float);
					cloud_.row_step = cloud_.point_step * cloud_.width;
					cloud_.fields.resize(numChannels);
					for (int i = 0; i < numChannels; i++) {
						std::string channelId[] = { "x", "y", "z", "intensity" };
						cloud_.fields[i].name = channelId[i];
						cloud_.fields[i].offset = i * sizeof(float);
						cloud_.fields[i].count = 1;
						cloud_.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
					}

					cloud_.data.resize(cloud_.row_step * cloud_.height);

					// Sehr hilfreich: https://answers.ros.org/question/191265/pointcloud2-access-data/
					// https://gist.github.com/yuma-m/b5dcce1b515335c93ce8
					// Copy to pointcloud
					int layer = 0;
					int baseLayer = 0;
					switch (numOfLayers)
					{
					case 1: // TIM571 etc.
						baseLayer = 0;
						break;
					case 4:

						baseLayer = -1;
						if (msg.header.seq == 250) layer = -1;
						else if (msg.header.seq == 0) layer = 0;
						else if (msg.header.seq == -250) layer = 1;
						else if (msg.header.seq == -500) layer = 2;
						elevationAngleDegree = this->parser_->getCurrentParamPtr()->getElevationDegreeResolution();
						elevationAngleDegree = elevationAngleDegree / 180.0 * M_PI;
						// 0.0436332 /*2.5 degrees*/;
						break;
					case 24: // Preparation for MRS6000
						printf("Seq: %d\n", msg.header.seq);
						baseLayer = -1;
						// -26.38
						// 
						layer = (msg.header.seq - (-2638)) / 125;
						layer += -1;
						/*							if (msg.header.seq == 250) layer = -1;
													else if (msg.header.seq == 0) layer = 0;
													else if (msg.header.seq == -250) layer = 1;
													else if (msg.header.seq == -500) layer = 2;
													*/
						elevationAngleDegree = this->parser_->getCurrentParamPtr()->getElevationDegreeResolution();
						elevationAngleDegree = elevationAngleDegree / 180.0 * M_PI;
						// 0.0436332 /*2.5 degrees*/;
						break;
					default:assert(0); break; // unsupported

					}

					for (size_t iEcho = 0; iEcho < numValidEchos; iEcho++)
					{
						float angle = config_.min_ang;
						int rangeNum = rangeTmp.size() / numEchos;
						for (size_t i = 0; i < rangeNum; i++)
						{
							geometry_msgs::Point32 point;
							float r = rangeTmp[iEcho * rangeNum + i];
							point.x = cos(angle) * r;
							point.y = sin(angle) * r;
							point.z = sin(layer * elevationAngleDegree /*2.5 degrees*/) * sqrt(point.x * point.x + point.y * point.y);

							//	cloud_.points[(layer - baseLayer) * msg.ranges.size() + i] = point;

							long adroff = i * (numChannels * (int)sizeof(float)) + (layer - baseLayer) * cloud_.row_step;
							adroff += iEcho * cloud_.row_step * numOfLayers;
							unsigned char *ptr = &(cloud_.data[0]) + adroff;

							float intensity = 0.0;
							if (config_.intensity)
							{
								intensity = msg.intensities[aiValidEchoIdx[iEcho] * rangeNum + i];

							}
							memcpy(ptr + 0, &(point.x), sizeof(float));
							memcpy(ptr + 4, &(point.y), sizeof(float));
							memcpy(ptr + 8, &(point.z), sizeof(float));
							memcpy(ptr + 12, &(intensity), sizeof(float));

							angle += msg.angle_increment;
							// Punktewerte werden in points abgelegt.
							// Die Ebeneninformation steht in channels[0].values
//							cloud_.channels[0].values[(layer - baseLayer) * msg.ranges.size() + i] = layer;
						}
						// Publish
						static int cnt = 0;
						int layerOff = (layer - baseLayer);

					}
					// if ( (msg.header.seq == 0) || (layerOff == 0)) // FIXEN!!!!
					if (msg.header.seq == 0)
					{  // pulish, if seq == 0 // true for every TIM5xx scan, true for every 4th for LMS1000 and MRS1104
					  //							cloud_.header.frame_id = msg.header.frame_id;
#ifndef _MSC_VER
						cloud_pub_.publish(cloud_);
#else
						printf("PUBLISH:\n");
#endif
					}
				}
			}
			// Start Point
			buffer_pos = dend + 1;
		} // end of while loop

		return ExitSuccess; // return success to continue looping
	}

	void SickScanCommon::check_angle_range(SickScanConfig &conf)
	{
		if (conf.min_ang > conf.max_ang)
		{
			ROS_WARN("Minimum angle must be greater than maximum angle. Adjusting min_ang.");
			conf.min_ang = conf.max_ang;
		}
	}

	void SickScanCommon::update_config(sick_scan::SickScanConfig &new_config, uint32_t level)
	{
		check_angle_range(new_config);
		config_ = new_config;
	}

	//************************************
	// Method:    tryCrc8
	// FullName:  sick_scan::tryCrc8
	// Access:    public 
	// Returns:   void
	// Qualifier: Example to show the calculation of CRC8 for binary commands. 
	//			  The sopas manual is not clear here.
	//************************************
	void tryCrc8()
	{
		unsigned char msgBlock[] =
		{ 0x02, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x17, 0x73, 0x4D, 0x4E, 0x20, 0x53, 0x65, 0x74, 0x41, 0x63, 0x63, 0x65, 0x73, 0x73, 0x4D, 0x6F, 0x64, 0x65, 0x20, 0x03, 0xF4, 0x72, 0x47, 0x44 };

		unsigned char xorVal = 0x00;
		int len = (sizeof(msgBlock) / sizeof(msgBlock[0]));
		int off = 8;
		for (int i = off; i < len; i++)
		{

			unsigned char val = msgBlock[i];
			xorVal ^= val;
		}
		printf("%02x\n", xorVal);


	}

	unsigned char sick_crc8(unsigned char *msgBlock, int len)
	{
		unsigned char xorVal = 0x00;
		int off = 0;
		for (int i = off; i < len; i++)
		{

			unsigned char val = msgBlock[i];
			printf("%02x ", val);

			xorVal ^= val;
		}
		printf("\n%02x\n", xorVal);
		return(xorVal);
	}

	int SickScanCommon::convertAscii2BinaryCmd(const char *requestAscii, std::vector<char>* requestBinary)
	{
		requestBinary->clear();
		if (requestAscii == NULL)
		{
			return(-1);
		}
		int cmdLen = strlen(requestAscii);
		if (cmdLen == 0)
		{
			return(-1);
		}
		if (requestAscii[0] != 0x02)
		{
			return(-1);
		}
		if (requestAscii[cmdLen - 1] != 0x03)
		{
			return(-1);
		}
		// Here we know, that the ascii format is correctly framed with <stx> .. <etx>
		for (int i = 0; i < 4; i++)
		{
			requestBinary->push_back(0x02);
		}

		unsigned long msgLen = cmdLen - 2; // without stx and etx

		for (int i = 0; i < 4; i++)
		{
			unsigned char bigEndianLen = msgLen & (0xFF << (3 - i) * 8);
			requestBinary->push_back((char)(bigEndianLen));
		}
		bool switchDoBinaryData = false;
		for (int i = 1; i <= (int)(msgLen); i++)  // STX DATA ETX --> 0 1 2
		{
			char c = requestAscii[i];
			if (switchDoBinaryData == true)
			{
				if (c >= '0' && c <= '9')
				{
					c -= '0';
				}
			}
			requestBinary->push_back(c);
			if (requestAscii[i] == 0x20) // space
			{
				switchDoBinaryData = true;
			}

		}

		unsigned char xorVal = 0x00;
		xorVal = sick_crc8((unsigned char *)(&((*requestBinary)[8])), requestBinary->size() - 8);
		requestBinary->push_back(xorVal);
		return(0);

	};

	bool SickScanCommon::switchToAuthorizeClient()
	{
		bool ret = false;
		int result = -1;
		std::vector<unsigned char> requestAccessModeAuthClientReply;
		result = sendSopasAndCheckAnswer(sopasCmdVec[CMD_SET_ACCESS_MODE_3].c_str(), &requestAccessModeAuthClientReply);
		std::string setAccessModeReplyStr = replyToString(requestAccessModeAuthClientReply);
		ret = result ? 0 : 1;
		return(ret);
	}

	bool SickScanCommon::stopScanData()
	{
		bool bRet = true;
		std::vector<unsigned char> scanDataReply;
		int result = sendSopasAndCheckAnswer(sopasCmdVec[CMD_STOP_SCANDATA].c_str(), &scanDataReply);
		if (result != 0)
		{
			bRet = false;
		}
		return(bRet);
	}

	bool SickScanCommon::startScanData()
	{
		bool bRet = true;
		std::vector<unsigned char> scanDataReply;
		int result = sendSopasAndCheckAnswer(sopasCmdVec[CMD_START_SCANDATA].c_str(), &scanDataReply);
		if (result != 0)
		{
			bRet = false;
		}
		return(bRet);
	}

	bool SickScanCommon::stopMeasurement()
	{

		bool bRet = true;
		int result;
		std::vector<unsigned char> stoppingMeasuremenReply;
		result = sendSOPASCommand(sopasCmdVec[CMD_STOP_MEASUREMENT].c_str(), &stoppingMeasuremenReply);
		if (result != 0)
		{
			ROS_ERROR("SOPAS - Error Stopping Measurement'.");
			diagnostics_.broadcast(getDiagnosticErrorCode(), "SOPAS - Error stooping measurement.");
		}

		const int MAX_TRIALS = 10; // wait for incoming message NOT containing  STOP Reply
		int traiLoopCnt = 0;
		for (traiLoopCnt = 0; traiLoopCnt < MAX_TRIALS; traiLoopCnt++)
		{
			std::string stoppingMeasuremenReplyStr = replyToString(stoppingMeasuremenReply);
			if (stoppingMeasuremenReplyStr.find("LMCstopmeas") == std::string::npos)
			{
				// not found - try to read next reply
				result = sendSOPASCommand(NULL, &stoppingMeasuremenReply);
			}
			else
			{
				break;
			}
		}
		if (traiLoopCnt == MAX_TRIALS)
		{
			ROS_ERROR("SOPAS - Error stopping scanner");
			bRet = false;

			diagnostics_.broadcast(getDiagnosticErrorCode(), "SOPAS - Error stopping scanner");
		}

		return(bRet);
	}

	bool SickScanCommon::run()
	{
		bool bRet = true;
		std::vector<unsigned char> runReply;
		int result = sendSopasAndCheckAnswer(sopasCmdVec[CMD_RUN].c_str(), &runReply);
		if (result != 0)
		{
			bRet = false;
		}
		return(bRet);
	}

	bool SickScanCommon::startMeasurement()
	{
		bool bRet = true;
		std::vector<unsigned char> startMeasurementReply;
		int result = sendSopasAndCheckAnswer(sopasCmdVec[CMD_START_MEASUREMENT].c_str(), &startMeasurementReply);
		if (result != 0)
		{
			bRet = false;
		}
		return(bRet);
	}

} /* namespace sick_scan */



