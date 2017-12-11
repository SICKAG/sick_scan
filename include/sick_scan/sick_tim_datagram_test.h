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

#ifndef SICK_TIM3XX_DATAGRAM_TEST_H_
#define SICK_TIM3XX_DATAGRAM_TEST_H_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#ifndef _MSC_VER
#include <dynamic_reconfigure/server.h>
#endif
#include <sick_scan/SickScanConfig.h>

#include "abstract_parser.h"

namespace sick_scan
{

class SickTimDatagramTest
{
public:
  SickTimDatagramTest(AbstractParser* parser);
  virtual ~SickTimDatagramTest();
  void check_angle_range(SickScanConfig &conf);
  void update_config(sick_scan::SickScanConfig &new_config, uint32_t level = 0);
  bool  testSetIpAddress(int argc, char **argv);

  bool testSetMeanFilter(int argc, char **argv);
private:
  ros::NodeHandle nh_;

  // publisher to "scan" topic
  ros::Publisher pub_;

  // subscriber to "datagram" topic
  ros::Subscriber sub_;
  void datagramCB(const std_msgs::String::ConstPtr &msg);

  // Dynamic Reconfigure
  SickScanConfig config_;
#ifndef _MSC_VER
  dynamic_reconfigure::Server<sick_scan::SickScanConfig> dynamic_reconfigure_server_;
#endif
  AbstractParser* parser_;
};

} /* namespace sick_scan */
#endif /* SICK_TIM3XX_DATAGRAM_TEST_H_ */
