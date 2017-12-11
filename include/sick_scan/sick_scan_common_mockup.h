/*
 * Copyright (C) 2015, Osnabrück University
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
 *  Created on: 30.01.2015
 *
 *      Author:
 *         Martin Günther <mguenthe@uos.de>
 *
 */

#ifndef SICK_TIM3XX_COMMON_MOCKUP_H_
#define SICK_TIM3XX_COMMON_MOCKUP_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _MSC_VER
#else
#include <libusb.h>
#endif
#include <std_msgs/String.h>

#include "sick_scan_common.h"

namespace sick_scan
{

class SickScanCommonMockup : public SickScanCommon
{
public:
  SickScanCommonMockup(SickGenericParser* parser);
  virtual ~SickScanCommonMockup();

protected:
  virtual int init_device();
  virtual int init_scanner();
  virtual int close_device();

  /// Send a SOPAS command to the device and print out the response to the console.
  virtual int sendSOPASCommand(const char* request, std::vector<unsigned char> * reply);

  /// Read a datagram from the device.
  /**
   * \param [in] receiveBuffer data buffer to fill
   * \param [in] bufferSize max data size to write to buffer (result should be 0 terminated)
   * \param [out] actual_length the actual amount of data written
   */
  virtual int get_datagram(unsigned char* receiveBuffer, int bufferSize, int* actual_length);

private:
  ros::NodeHandle nh_;

  // subscriber to "datagram" topic
  ros::Subscriber sub_;

  std_msgs::String::ConstPtr datagram_msg_;

  void datagramCB(const std_msgs::String::ConstPtr &msg);
};

} /* namespace sick_scan */
#endif /* SICK_TIM3XX_COMMON_MOCKUP_H_ */
