/*
 * Copyright (C) 2013, Osnabrück University
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
 *  Created on: 21.08.2013
 *
 *      Author: Martin Günther <mguenthe@uos.de>
 *
 */



#ifdef _MSC_VER
#define _WIN32_WINNT 0x0501
#pragma warning(disable: 4996)
#pragma warning(disable: 4267)
#endif

#ifndef _MSC_VER
#include <sick_scan/sick_scan_common_usb.h>
#endif
#include <sick_scan/sick_scan_common_tcp.h>
#include <sick_scan/sick_scan_common_mockup.h>

#include <sick_scan/sick_generic_parser.h>

#ifdef _MSC_VER
#include "sick_scan/rosconsole_simu.hpp"
#endif
#define _USE_MATH_DEFINES
#include <math.h>
#include "string"
#include <stdio.h>
#include <stdlib.h>

#include <sick_scan/sick_tim_datagram_test.h>
namespace sick_scan
{

	SickTimDatagramTest::SickTimDatagramTest(AbstractParser* parser) :
		parser_(parser)
	{
#ifndef _MSC_VER
		//dynamic_reconfigure_server_.getConfigDefault(config_);
		dynamic_reconfigure::Server<sick_scan::SickScanConfig>::CallbackType f;
		f = boost::bind(&sick_scan::SickTimDatagramTest::update_config, this, _1, _2);
		dynamic_reconfigure_server_.setCallback(f);
#endif
		pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan_from_datagram", 1000);
		sub_ = nh_.subscribe("datagram", 1, &SickTimDatagramTest::datagramCB, this);
	}

	SickTimDatagramTest::~SickTimDatagramTest()
	{
		delete parser_;
	}

	void SickTimDatagramTest::datagramCB(const std_msgs::String::ConstPtr &datagram_msg)
	{
		sensor_msgs::LaserScan scan_msg;

		std::vector<char> str(datagram_msg->data.begin(), datagram_msg->data.end());
		str.push_back('\0');
		int numEchos = 0;
		int success = parser_->parse_datagram(&str[0], datagram_msg->data.length(), config_, scan_msg, numEchos);
		if (success == ExitSuccess)
			pub_.publish(scan_msg);
		else
			ROS_ERROR("parse_datagram returned %d!", success);
	}

	void SickTimDatagramTest::check_angle_range(SickScanConfig &conf)
	{
		if (conf.min_ang > conf.max_ang)
		{
			ROS_WARN("Minimum angle must be greater than maximum angle. Adjusting min_ang.");
			conf.min_ang = conf.max_ang;
		}
	}

	void SickTimDatagramTest::update_config(sick_scan::SickScanConfig &new_config, uint32_t level)
	{
		check_angle_range(new_config);
		config_ = new_config;
	}




	bool  SickTimDatagramTest::testSetIpAddress(int argc, char **argv)
	{
		bool ret = false;
		//test code here
		return(ret);
	}

	bool  SickTimDatagramTest::testSetMeanFilter(int argc, char **argv)
	{
		bool ret = true;
		//test code here
		return(ret);
	}


	class SickScanTest
	{
	public:
		std::string testId;
		bool(SickTimDatagramTest::*testFunc)(int argc, char **);
	};

} /* namespace sick_scan */

static bool getTagVal(std::string tagVal, std::string& tag, std::string& val)
{
	bool ret = false;
	std::size_t pos;
	pos = tagVal.find(":=");
	tag = "";
	val = "";
	if (pos == std::string::npos)
	{
		ret = false;
	}
	else
	{
		tag = tagVal.substr(0, pos);
		val = tagVal.substr(pos + 2);
		ret = true;
	}
	return(ret);
}

typedef  bool (sick_scan::SickTimDatagramTest::*TestMemFn)(int argc, char *argv[]);

int main(int argc, char **argv)
{
	std::string tag;
	std::string val;
	std::string scannerName = "sick_mrs_1xxx";
	bool doInternalDebug = false;
	// for internal debug set "__internalDebug:=1"
	for (int i = 0; i < argc; i++)
	{
		std::string s = argv[i];
		if (getTagVal(s, tag, val))
		{
			if (tag.compare("__internalDebug") == 0)
			{
				int debugState = 0;
				sscanf(val.c_str(), "%d", &debugState);
				if (debugState > 0)
				{
					doInternalDebug = true;
				}
			}
		}
	}
	ros::init(argc, argv, scannerName);
	ros::NodeHandle nhPriv("~");

	if (doInternalDebug)
	{
		// set all parameters, which are necessary for debugging without using roslaunch. Just start roscore at the beginning of your debug session
		int tmpactive_echos = 7;
		bool tmpauto_reboot = true;
		std::string tmpframe_id = "laser";
		//    std::string tmphostname="192.168.0.1";  // TIM 571 Test
		std::string tmphostname = "192.168.0.21";
		bool tmpintensity = true;
		double tmpminang = -30.00 / 180.0 * M_PI;
		double tmpmaxang = 30.0 / 180.0 * M_PI;
		std::string tmpport = "2112";
		double tmprange_min = 0.01;
		double tmprange_max = 25.0;
		int tmpskip = 0;
		double tmptime_offset = -0.001;
		double tmptimelimit = 5;

		int tmpVerboseLevel = 0;  // write debug file for scan data
		nhPriv.setParam("verboseLevel", tmpVerboseLevel);
		nhPriv.setParam("active_echos", tmpactive_echos);
		nhPriv.setParam("auto_reboot", tmpauto_reboot);
		nhPriv.setParam("hostname", tmphostname);
		nhPriv.setParam("frame_id", tmpframe_id);
		nhPriv.setParam("intensity", tmpintensity);
		nhPriv.setParam("min_ang", tmpminang);
		nhPriv.setParam("max_ang", tmpmaxang);
		nhPriv.setParam("port", tmpport);
		nhPriv.setParam("range_min", tmprange_min);
		nhPriv.setParam("range_max", tmprange_max);
		nhPriv.setParam("skip", tmpskip);
		nhPriv.setParam("time_offset", tmptime_offset);
		nhPriv.setParam("timelimit", tmptimelimit);

	}
	/*
	* Dump of ROS config
	* sick_mrs_1000: {
	* active_echos: 65535,
	* auto_reboot: true,
	* frame_id: laser, hostname: 192.168.0.1,
	intensity: true, max_ang: 2.35619449019, minAng: -2.0, min_ang: -2.0, port: '2112',
	range_max: 25.0, skip: 0, time_offset: -0.001, timelimit: 5}
	*/
	// check for TCP - use if ~hostname is set.
	bool useTCP = false;
	std::string hostname;
	if (nhPriv.getParam("hostname", hostname)) {
		useTCP = true;
	}
	std::string port;
	nhPriv.param<std::string>("port", port, "2112");

	int timelimit;
	nhPriv.param("timelimit", timelimit, 5);

	bool subscribe_datagram;
	int device_number;
	nhPriv.param("subscribe_datagram", subscribe_datagram, false);
	nhPriv.param("device_number", device_number, 0);


	sick_scan::SickGenericParser* parser = new sick_scan::SickGenericParser(scannerName);

	//------------------ TESTING -------------------------

	double param;
	if (nhPriv.getParam("range_min", param))
	{
		parser->set_range_min(param);
	}
	if (nhPriv.getParam("range_max", param))
	{
		parser->set_range_max(param);
	}
	if (nhPriv.getParam("time_increment", param))
	{
		parser->set_time_increment(param);
	}

	sick_scan::SickScanCommon* s = NULL;

	int result = sick_scan::ExitError;
	while (ros::ok())
	{
		// Atempt to connect/reconnect
		delete s;
		if (subscribe_datagram)
			s = new sick_scan::SickScanCommonMockup(parser);
		else if (useTCP)
			s = new sick_scan::SickScanCommonTcp(hostname, port, timelimit, parser);
#ifndef _MSC_VER
		else
			s = new sick_scan::SickScanCommonUsb(parser, device_number);
#endif

		result = s->init();
		s->switchToAuthorizeClient();
		// HIER TESTCODE TESTCASES
		s->stopScanData();
		// s->stopMeasurement();
		ROS_DEBUG("\n");
		ROS_DEBUG("=============================================\n");
		ROS_DEBUG("| UNIT-TEST                                 | \n");
		ROS_DEBUG("=============================================\n");
		bool ret;
		ret = s->testsetParticleFilter();
		ROS_DEBUG("TEST: SetParticleFilter: [%s]\n", ret ? "PASSED" : "FAILED");
		ret = s->testsetMeanFilter();
		ROS_DEBUG("TEST: SetMeanFilter: [%s]\n", ret ? "PASSED" : "FAILED");
		ret = s->testsetAligmentMode();
		ROS_DEBUG("TEST: SetAligmentMode: [%s]\n", ret ? "PASSED" : "FAILED");
		ret = s->testsetActivateStandBy();
		ROS_DEBUG("TEST: SetActivateStandBy: [%s]\n", ret ? "PASSED" : "FAILED");
		ret = s->testsetApplicationMode();
		ROS_DEBUG("TEST: testsetApplicationMode: [%s]\n", ret ? "PASSED" : "FAILED");



		// insert here parsing of argument for test decision

		sick_scan::SickGenericParser* parser = new sick_scan::SickGenericParser(SICK_SCANNER_MRS_1000_NAME);
		sick_scan::SickTimDatagramTest s(parser);

		ROS_DEBUG("Start Default Spinning procedure of ROS\n");
		ros::spin();

		return 0;
	}
}
