/**
* \file
* \brief Laser Scanner communication
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
#define _WIN32_WINNT 0x0501
#pragma warning(disable: 4996)
#pragma warning(disable: 4267)
#endif

#ifndef _MSC_VER


#endif
#include <sick_scan/sick_scan_common_tcp.h>

#include <sick_scan/sick_generic_parser.h>

#ifdef _MSC_VER
#include "sick_scan/rosconsole_simu.hpp"
#endif
#define _USE_MATH_DEFINES
#include <math.h>
#include "string"
#include <stdio.h>
#include <stdlib.h>

bool getTagVal(std::string tagVal, std::string& tag, std::string& val)
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

/*!
\brief Internal Startup routine. In the future the scannerName should be coded in a specific param-Entry 
       like "scannerType"
       
\param argc: Number of Arguments
\param argv: Argument variable
\param nodeName name of the ROS-node
\return exit-code
\sa main
*/
int mainGenericLaser(int argc, char **argv, std::string nodeName)
{
	std::string tag;
	std::string val;

	bool doInternalDebug = false;

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

	ros::init(argc, argv, nodeName);  // scannerName holds the node-name
	ros::NodeHandle nhPriv("~");

	std::string scannerName;
	if (false == nhPriv.getParam("scanner_type", scannerName))
	{
		ROS_ERROR("cannot find parameter ""scanner_type"" in the param set. Please specify scanner_type.");
		ROS_ERROR("Try to set sick_tim_5xx as fallback.\n");
		scannerName = "sick_tim_5xx";
	}


	if (doInternalDebug)
	{
#ifdef _MSC_VER
		nhPriv.setParam("name", scannerName);
		rossimu_settings(nhPriv);  // just for tiny simulations under Visual C++
#endif
	}

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

	double param;
	char colaDialectId = 'A'; // A or B (Ascii or Binary)

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

	/*
	 *  Check, if parameter for protocol type is set
	 */
	bool use_binary_protocol = true;
	if (true == nhPriv.getParam("use_binary_protocol", use_binary_protocol))
	{
		ROS_INFO("Found sopas_protocol_type param overwriting default protocol:");
		if (use_binary_protocol==true)
		{
			ROS_INFO("Binary protocol activated");
		}
		else
		{
			if(parser->getCurrentParamPtr()->getNumberOfLayers()>4)
			{
				nhPriv.setParam("sopas_protocol_type", true);
				use_binary_protocol=true;
				ROS_WARN("This scanner type does not support ASCII communication.\n"
								 "Binary communication has been activated.\n"
								 "The parameter \"sopas_protocol_type\" has been set to \"True\".");
			}
			else
			{
				ROS_INFO("ASCII protocol activated");
			}

		}
		parser->getCurrentParamPtr()->setUseBinaryProtocol(use_binary_protocol);
	}


	if (parser->getCurrentParamPtr()->getUseBinaryProtocol())
	{
		colaDialectId = 'B';
	}
	else
	{
		colaDialectId = 'A';
	}

	sick_scan::SickScanCommon* s = NULL;

	int result = sick_scan::ExitError;
	while (ros::ok())
	{
    ROS_INFO("Start initialising scanner ...");
		// attempt to connect/reconnect
		delete s;  // disconnect scanner
        if (useTCP)
		  s = new sick_scan::SickScanCommonTcp(hostname, port, timelimit, parser, colaDialectId);
        else
        {
           ROS_ERROR("TCP is not switched on. Probably hostname or port not set. Use roslaunch to start node.");
           exit(-1);
        }

		result = s->init();

		sick_scan::SickScanConfig cfg;

		while (ros::ok() && (result == sick_scan::ExitSuccess))
		{
			ros::spinOnce();
			result = s->loopOnce();
		}

		if (result == sick_scan::ExitFatal)
			return result;

	}

	delete s; // close connnect
	delete parser; // close parser
	return result;
}
