/*
 * @brief Implementation of ROS messages for sick_scan
 *
 * Copyright (C) 2021, Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2021, SICK AG, Waldkirch
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
 *  Created on: 12.01.2021
 *
 *      Authors:
 *         Michael Lehning <michael.lehning@lehning.de>
 *
 * Based on the TiM communication example by SICK AG.
 *
 */

#include "sick_scan/sick_scan_messages.h"

typedef uint8_t* byte_ptr;
template<typename T> static bool readBinaryBuffer(byte_ptr& buffer, int & bufferlen, T& value)
{
    if(sizeof(value) > bufferlen)
    {
        ROS_ERROR_STREAM("## ERROR SickScanMessages::readBinaryBuffer(): bufferlen=" << bufferlen << " byte, " << sizeof(value) << " byte required.");
        return false;
    }
    memcpy(&value, buffer, sizeof(value));
    swap_endian((unsigned char *) &value, sizeof(value));
    buffer += sizeof(value);
    bufferlen -= (int)sizeof(value);
    return true;
}

sick_scan::SickScanMessages::SickScanMessages(ros::NodeHandle* nh)
{
}

sick_scan::SickScanMessages::~SickScanMessages()
{
}

/*
 * @brief parses and converts a lidar LIDoutputstate message to a ros LIDoutputstate message
 * 
 * Example LIDoutputstate message from 20210111_sick_tim781s_mon_elephant.pcapng.json:
 * "tcp.description": ".......4sSN LIDoutputstate .................................E",
 * "tcp.payload": "02:02:02:02:00:00:00:34:73:53:4e:20:4c:49:44:6f:75:74:70:75:74:73:74:61:74:65:20:00:00:00:00:00:00:01:00:00:01:26:00:00:00:01:22:00:00:00:00:00:01:00:00:00:04:02:00:00:00:00:00:00:45"
 * 33 byte LIDoutputstate payload after "sSN LIDoutputstate ": "00:00:00:00:00:00:01:00:00:01:26:00:00:00:01:22:00:00:00:00:00:01:00:00:00:04:02:00:00:00:00:00:00"
 * 
 * @param[in] timeStamp timestamp on receiving the lidar message
 * @param[in] receiveBuffer byte array of lidar message
 * @param[in] receiveLength size of lidar message in byte
 * @param[in] useBinaryProtocol binary lidar message (true, Cola-B) or ascii lidar message (false, Cola-A)
 * @param[in] frame_id frame id of output message
 * @param[out] output_msg converted output message
 * 
 * @return true on success, false on error
 */
bool sick_scan::SickScanMessages::parseLIDoutputstateMsg(const ros::Time& timeStamp, uint8_t* receiveBuffer, int receiveLength, bool useBinaryProtocol, const std::string& frame_id, sick_scan::LIDoutputstateMsg& output_msg)
{
    if(useBinaryProtocol)
    {
        // parse and convert LIDoutputstate message
        int msg_start_idx = 27;  // 4 byte STX + 4 byte payload_length + 19 byte "sSN LIDoutputstate " = 27 byte
        int msg_parameter_length = receiveLength - msg_start_idx - 1; // start bytes + 1 byte CRC
        int msg_output_states_length = msg_parameter_length - 18; // 2 byte version + 4 byte system + 12 byte timestamp = 18 byte
        int number_of_output_states = msg_output_states_length / 5; // each output state has 1 byte state enum + 4 byte counter = 5 byte
        if((msg_output_states_length % 5) != 0 || number_of_output_states <= 0)
        {
            ROS_ERROR_STREAM("## ERROR SickScanMessages::parseLIDoutputstateMsg(): received " << receiveLength << " byte with " << msg_output_states_length << " byte states, expected multiple of 5 (" << __FILE__ << ":" << __LINE__ << ")");
            return false;
        }
        receiveBuffer += msg_start_idx;
        receiveLength -= msg_start_idx;
        if( !readBinaryBuffer(receiveBuffer, receiveLength, output_msg.version_number)
         || !readBinaryBuffer(receiveBuffer, receiveLength, output_msg.system_counter))
        {
            ROS_ERROR_STREAM("## ERROR SickScanMessages::parseLIDoutputstateMsg(): error parsing version_number and system_counter (" << __FILE__ << ":" << __LINE__ << ")");
            return false;
        }
        output_msg.output_state.reserve(number_of_output_states);
        output_msg.output_count.reserve(number_of_output_states);
        for(int state_cnt = 0; state_cnt < number_of_output_states; state_cnt++)
        {
            uint8_t output_state;
            uint32_t output_count;
            if( !readBinaryBuffer(receiveBuffer, receiveLength, output_state)
             || !readBinaryBuffer(receiveBuffer, receiveLength, output_count))
            {
                ROS_ERROR_STREAM("## ERROR SickScanMessages::parseLIDoutputstateMsg(): error parsing version_number and system_counter (" << __FILE__ << ":" << __LINE__ << ")");
                return false;
            }
            output_msg.output_state.push_back(output_state);
            output_msg.output_count.push_back(output_count);
        }
        if( !readBinaryBuffer(receiveBuffer, receiveLength, output_msg.time_state)
         || !readBinaryBuffer(receiveBuffer, receiveLength, output_msg.year)
         || !readBinaryBuffer(receiveBuffer, receiveLength, output_msg.month)
         || !readBinaryBuffer(receiveBuffer, receiveLength, output_msg.day)
         || !readBinaryBuffer(receiveBuffer, receiveLength, output_msg.hour)
         || !readBinaryBuffer(receiveBuffer, receiveLength, output_msg.minute)
         || !readBinaryBuffer(receiveBuffer, receiveLength, output_msg.second)
         || !readBinaryBuffer(receiveBuffer, receiveLength, output_msg.microsecond))
        {
            ROS_ERROR_STREAM("## ERROR SickScanMessages::parseLIDoutputstateMsg(): error parsing timestamp (" << __FILE__ << ":" << __LINE__ << ")");
            return false;
        }
        output_msg.header.stamp = timeStamp;
        output_msg.header.seq = 0;
        output_msg.header.frame_id = frame_id;

        std::stringstream state_str;
        for(int state_cnt = 0; state_cnt < number_of_output_states; state_cnt++)
            state_str << "state[" << state_cnt << "]: " << (uint32_t)output_msg.output_state[state_cnt] << ", count=" << (uint32_t)output_msg.output_count[state_cnt] << "\n";
        ROS_DEBUG_STREAM("SickScanMessages::parseLIDoutputstateMsg():\n"
            << "version_number: " << (uint32_t)output_msg.version_number << ", system_counter: " << (uint32_t)output_msg.system_counter << "\n"
            << state_str.str()
            << "time state: " << (uint32_t)output_msg.time_state
            << ", date: " << std::setfill('0') << std::setw(4) << (uint32_t)output_msg.year << "-" << std::setfill('0') << std::setw(2) << (uint32_t)output_msg.month << "-" << std::setfill('0') << std::setw(2) << (uint32_t)output_msg.day
            << ", time: " << std::setfill('0') << std::setw(2) << (uint32_t)output_msg.hour << ":" << std::setfill('0') << std::setw(2) << (uint32_t)output_msg.minute << ":" << std::setfill('0') << std::setw(2) << (uint32_t)output_msg.second << "." << std::setfill('0') << std::setw(6) << (uint32_t)output_msg.microsecond);

        return true;
    }
    else
    {
        ROS_ERROR_STREAM("## ERROR SickScanMessages::parseLIDoutputstateMsg not implemented for Cola-A messages(" << __FILE__ << ":" << __LINE__ << ")");        
    }
    return false;
}

/*
 * @brief parses and converts a lidar LFErec message to a ros LFErec message
 * 
 * Example LIDoutputstate message from 20210111_sick_tim781s_lferec_elephant.pcapng.json:
 * "tcp.description": "........sSN LFErec ........................0..............3...........................0..............4....P......................0..............35.....",
 * "tcp.payload": "02:02:02:02:00:00:00:8e:73:53:4e:20:4c:46:45:72:65:63:20:00:03:00:01:01:00:00:00:00:3f:80:00:00:00:00:00:00:00:00:0d:05:ff:f9:22:30:00:00:00:00:00:00:00:00:01:07:b2:01:01:08:33:2e:00:09:ac:90:00:01:02:00:00:00:00:3f:80:00:00:00:00:00:00:00:00:0d:05:ff:f9:22:30:01:00:00:00:00:00:00:00:01:07:b2:01:01:08:34:02:00:06:9f:50:00:01:03:00:00:00:00:3f:80:00:00:00:00:00:00:00:00:0d:05:ff:f9:22:30:01:00:00:00:00:00:00:00:01:07:b2:01:01:08:33:35:00:09:ac:90:ac"
 * 131 byte LFErec payload after "sSN LFErec ": "00:03:00:01:01:00:00:00:00:3f:80:00:00:00:00:00:00:00:00:0d:05:ff:f9:22:30:00:00:00:00:00:00:00:00:01:07:b2:01:01:08:33:2e:00:09:ac:90:00:01:02:00:00:00:00:3f:80:00:00:00:00:00:00:00:00:0d:05:ff:f9:22:30:01:00:00:00:00:00:00:00:01:07:b2:01:01:08:34:02:00:06:9f:50:00:01:03:00:00:00:00:3f:80:00:00:00:00:00:00:00:00:0d:05:ff:f9:22:30:01:00:00:00:00:00:00:00:01:07:b2:01:01:08:33:35:00:09:ac:90"
 * LFErec payload starts with 16 byte fields_number "00:03" -> 3 fields, each field has (131-2)/3 = 43 byte
 * Each field contains a field_index (1,2,3) and field_result (0: invalid/incorrect, 1: free/clear, 2: infringed)
 * Note: field indices are sorted in reverse order, i.e. with 2 fields configured, we have typically
 * output_msg.fields[0].field_index = 1, output_msg.fields[0].field_result_mrs = 0 (invalid)
 * output_msg.fields[1].field_index = 2, output_msg.fields[1].field_result_mrs = 1 or 2 (clear=1 or infringed=2)
 * output_msg.fields[2].field_index = 3, output_msg.fields[2].field_result_mrs = 1 or 2 (clear=1 or infringed=2)
 * 
 * @param[in] timeStamp timestamp on receiving the lidar message
 * @param[in] receiveBuffer byte array of lidar message
 * @param[in] receiveLength size of lidar message in byte
 * @param[in] useBinaryProtocol binary lidar message (true, Cola-B) or ascii lidar message (false, Cola-A)
 * @param[in] frame_id frame id of output message
 * @param[out] output_msg converted output message
 * 
 * @return true on success, false on error
 */
bool sick_scan::SickScanMessages::parseLFErecMsg(const ros::Time& timeStamp, uint8_t* receiveBuffer, int receiveLength, bool useBinaryProtocol, const std::string& frame_id, sick_scan::LFErecMsg& output_msg)
{
    if(useBinaryProtocol)
    {
        // parse and convert LFErec messages, see https://github.com/SICKAG/libsick_ldmrs/blob/master/src/sopas/LdmrsSopasLayer.cpp lines 1414 ff.
        int msg_start_idx = 19;  // 4 byte STX + 4 byte payload_length + 11 byte "sSN LFErec " = 19 byte
        receiveBuffer += msg_start_idx;
        receiveLength -= msg_start_idx;

        output_msg.fields_number = 0;
        if(!readBinaryBuffer(receiveBuffer, receiveLength, output_msg.fields_number) || output_msg.fields_number <= 0)
        {
            ROS_ERROR_STREAM("## ERROR SickScanMessages::parseLFErecMsg(): parse error, fields number = " << output_msg.fields_number<< " should be greater 0 (" << __FILE__ << ":" << __LINE__ << ")");
            return false;
        }
        output_msg.fields.reserve(output_msg.fields_number);
        for(int field_idx = 0; field_idx < output_msg.fields_number; field_idx++)
        {
            sick_scan::LFErecFieldMsg field_msg;
            uint16_t dummies[3] = {0};
            if( !readBinaryBuffer(receiveBuffer, receiveLength, field_msg.version_number)
             || !readBinaryBuffer(receiveBuffer, receiveLength, field_msg.field_index)
             || !readBinaryBuffer(receiveBuffer, receiveLength, field_msg.sys_count)
             || !readBinaryBuffer(receiveBuffer, receiveLength, field_msg.dist_scale_factor)
             || !readBinaryBuffer(receiveBuffer, receiveLength, field_msg.dist_scale_offset)
             || !readBinaryBuffer(receiveBuffer, receiveLength, field_msg.angle_scale_factor)
             || !readBinaryBuffer(receiveBuffer, receiveLength, field_msg.angle_scale_offset)
             || !readBinaryBuffer(receiveBuffer, receiveLength, field_msg.field_result_mrs)
             || !readBinaryBuffer(receiveBuffer, receiveLength, dummies[0])
             || !readBinaryBuffer(receiveBuffer, receiveLength, dummies[1])
             || !readBinaryBuffer(receiveBuffer, receiveLength, dummies[2])
             || !readBinaryBuffer(receiveBuffer, receiveLength, field_msg.time_state)
             || !readBinaryBuffer(receiveBuffer, receiveLength, field_msg.year)
             || !readBinaryBuffer(receiveBuffer, receiveLength, field_msg.month)
             || !readBinaryBuffer(receiveBuffer, receiveLength, field_msg.day)
             || !readBinaryBuffer(receiveBuffer, receiveLength, field_msg.hour)
             || !readBinaryBuffer(receiveBuffer, receiveLength, field_msg.minute)
             || !readBinaryBuffer(receiveBuffer, receiveLength, field_msg.second)
             || !readBinaryBuffer(receiveBuffer, receiveLength, field_msg.microsecond))
            {
                ROS_ERROR_STREAM("## ERROR SickScanMessages::parseLFErecMsg(): parse error field " << field_idx << " (" << __FILE__ << ":" << __LINE__ << ")");
                return false;
            }
            output_msg.fields.push_back(field_msg);
        }
        if(output_msg.fields.size() != output_msg.fields_number)
        {
            ROS_ERROR_STREAM("## ERROR SickScanMessages::parseLFErecMsg(): parse error, fields number = " << output_msg.fields_number << ", but " << output_msg.fields.size() << " fields parsed (" << __FILE__ << ":" << __LINE__ << ")");
            return false;
        }
        if(receiveLength != 1) // 1 byte CRC still in receiveBuffer
        {
            ROS_ERROR_STREAM("## ERROR SickScanMessages::parseLFErecMsg(): parse error, " << receiveLength << " bytes in buffer after decoding all fields, should be 0 (" << __FILE__ << ":" << __LINE__ << ")");
            return false;
        }

        output_msg.header.stamp = timeStamp;
        output_msg.header.seq = 0;
        output_msg.header.frame_id = frame_id;

        std::stringstream fields_str;
        for(int field_idx = 0; field_idx < output_msg.fields.size(); field_idx++)
        {
            sick_scan::LFErecFieldMsg& field_msg = output_msg.fields[field_idx];
            if(field_idx > 0)
                fields_str << "\n";
            fields_str << "field[" << field_idx << "]: idx=" << (uint32_t)field_msg.field_index 
                << ", dist_scale_factor=" << field_msg.dist_scale_factor
                << ", dist_scale_offset=" << field_msg.dist_scale_offset
                << ", angle_scale_factor=" << field_msg.angle_scale_factor
                << ", angle_scale_offset=" << field_msg.angle_scale_offset
                << ", field_result_mrs=" << (uint32_t)field_msg.field_result_mrs
                << ", time state: " << (uint32_t)field_msg.time_state
                << ", date: " << std::setfill('0') << std::setw(4) << (uint32_t)field_msg.year << "-" << std::setfill('0') << std::setw(2) << (uint32_t)field_msg.month << "-" << std::setfill('0') << std::setw(2) << (uint32_t)field_msg.day
                << ", time: " << std::setfill('0') << std::setw(2) << (uint32_t)field_msg.hour << ":" << std::setfill('0') << std::setw(2) << (uint32_t)field_msg.minute << ":" << std::setfill('0') << std::setw(2) << (uint32_t)field_msg.second 
                << "." << std::setfill('0') << std::setw(6) << (uint32_t)field_msg.microsecond;
        }
        ROS_DEBUG_STREAM("SickScanMessages::parseLFErecMsg(): LFErec with " << output_msg.fields.size() << " fields:\n" << fields_str.str());

        return true;
    }
    else
    {
        ROS_ERROR_STREAM("## ERROR SickScanMessages::parseLFErecMsg not implemented for Cola-A messages (" << __FILE__ << ":" << __LINE__ << ")");        
    }
    return false;
}
