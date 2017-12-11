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
 *  Created on: 14.11.2013
 *
 *      Author: Martin Günther <mguenthe@uos.de>
 *
 */

#ifndef SICK_GENERIC_PARSER_H_
#define SICK_GENERIC_PARSER_H_

#define SICK_SCANNER_LMS_1000_NAME "sick_lms_1000"
#define SICK_SCANNER_MRS_1000_NAME "sick_mrs_1000"
#define SICK_SCANNER_TIM_5XX_NAME "sick_tim_5xx"
#define SICK_SCANNER_MRS_6000_NAME "sick_mrs_6000"
#include "abstract_parser.h"

#include "sensor_msgs/LaserScan.h"

// namespace sensor_msgs
namespace sick_scan
{
	class ScannerBasicParam
	{
	public:
		void setScannerName(std::string _s);
		std::string getScannerName(void);
		void setNumberOfLayers(int _layerNum);
		int getNumberOfLayers(void);
		void setNumberOfShots(int _shots);
		int getNumberOfShots(void);
		void setNumberOfMaximumEchos(int _maxEchos);
		int getNumberOfMaximumEchos(void);
		void setAngularDegreeResolution(double _res);
		void setElevationDegreeResolution(double _elevRes); 
		double getElevationDegreeResolution(void);
		double getAngularDegreeResolution(void);
		ScannerBasicParam();
	private:
		std::string scannerName;
		int numberOfLayers;
		int numberOfShots;
		int numberOfMaximumEchos;
		double elevationDegreeResolution;
		double angleDegressResolution;
	};



	class SickGenericParser : public AbstractParser
	{
	public:
		SickGenericParser(std::string scannerType);
		virtual ~SickGenericParser();

		virtual int parse_datagram(char* datagram, size_t datagram_length, SickScanConfig &config,
			sensor_msgs::LaserScan &msg, int &numEchos, int& echoMask);


		void set_range_min(float min);
		void set_range_max(float max);
		void set_time_increment(float time);
		void setScannerType(std::string s);
		std::string getScannerType(void);
		int lookUpForAllowedScanner(std::string scannerName);
		void setCurrentParamPtr(ScannerBasicParam* _ptr);
		ScannerBasicParam *getCurrentParamPtr();


		int checkForDistAndRSSI(std::vector<char *>& fields, int expected_number_of_data, int& distNum, int& rssiNum, std::vector<float>& distVal, std::vector<float>& rssiVal,int& distMask);


	private:
		float override_range_min_, override_range_max_;
		float override_time_increment_;
		std::string scannerType;
		std::vector<std::string> allowedScannerNames;
		std::vector<ScannerBasicParam> basicParams;
		ScannerBasicParam *currentParamSet;
	};

} /* namespace sick_scan */
#endif /* SICK_GENERIC_PARSER_H_ */
