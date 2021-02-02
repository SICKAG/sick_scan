/*
 * @brief Implementation of object markers for sick_scan
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
 *  Created on: 13.01.2021
 *
 *      Authors:
 *         Michael Lehning <michael.lehning@lehning.de>
 *
 */

#include "sick_scan/sick_scan_marker.h"

static void monFieldToCarthesian(float range, float angle_rad, float& x, float& y)
{
    y = -range * std::cos(angle_rad); // y_ros = -x_sick = -range * std::cos(angle_rad)
    x = range * std::sin(angle_rad);  // x_ros = +y_sick = +range * std::sin(angle_rad)
}

static std_msgs::ColorRGBA color(float r, float g, float b, float a = 0.5f)
{
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}

static std_msgs::ColorRGBA red(void)
{
    return color(1.0f, 0.0f, 0.0f);
}

static std_msgs::ColorRGBA green(void) // free fields
{
    return color(0.0f, 1.0f, 0.0f);
}

static std_msgs::ColorRGBA blue(void)
{
    return color(0.0f, 0.0f, 1.0f);
}

static std_msgs::ColorRGBA yellow(void) // infringed fields
{
    return color(1.0f, 1.0f, 0.0f);
}

static std_msgs::ColorRGBA gray(void) // invalid fields (default)
{
    return color(0.5f, 0.5f, 0.5f);
}

sick_scan::SickScanMarker::SickScanMarker(ros::NodeHandle* nh, const std::string & marker_topic, const std::string & marker_frame_id)
: m_scan_mon_fieldset(0)
{
    if(nh)
    {
        m_frame_id = marker_frame_id.empty() ? "/cloud" : marker_frame_id;
        m_marker_publisher = nh->advertise<visualization_msgs::MarkerArray>(marker_topic.empty() ? "sick_scan/marker" : marker_topic, 1);
    }
}

sick_scan::SickScanMarker::~SickScanMarker()
{
}

void sick_scan::SickScanMarker::updateMarker(const std::vector<SickScanMonField>& fields, int fieldset)
{
    m_scan_mon_fields = fields;
    m_scan_mon_fieldset = fieldset; // todo: query selected field set - maybe "sRN uiSelectedFieldNum" ??? "sRA uiSelectedFieldNum" returns 0 in 001_sopas_et_binary_startup.pcapng.json
    std::vector<FieldInfo> default_fields = {FieldInfo(0,0,"-","3",gray()), FieldInfo(1,0,"-","2",gray()), FieldInfo(2,0,"-","1",gray())};
    m_scan_mon_field_marker = createMonFieldMarker(default_fields);
    m_scan_mon_field_legend = createMonFieldLegend(default_fields);
    // m_scan_fieldset_legend = createMonFieldsetLegend(0);
    // m_scan_outputstate_legend = createOutputStateLegend({"0", "0", "0"}, {"-", "-", "-"}, {gray(), gray(), gray()}); // only if outputstates active, i.e. after updateMarker(LIDoutputstateMsg)
    publishMarker();
}

void sick_scan::SickScanMarker::updateMarker(sick_scan::LIDoutputstateMsg& msg)
{
    SickScanFieldMonSingleton *fieldMon = SickScanFieldMonSingleton::getInstance();
    if(fieldMon)
    {
        m_scan_mon_fieldset = fieldMon->getActiveFieldset();
        ROS_DEBUG_STREAM("SickScanMarker: active_fieldset = " << fieldMon->getActiveFieldset());
    }
    int num_devices = std::min(msg.output_count.size(), msg.output_state.size());
    std::vector<std::string> output_state(num_devices);
    std::vector<std::string> output_count(num_devices);
    std::vector<std_msgs::ColorRGBA> output_colors(num_devices);
    for(int field_idx = 0; field_idx < num_devices; field_idx++)
    {
        int count = msg.output_count[field_idx];
        int state = msg.output_state[field_idx];
        output_state[field_idx] = std::to_string(state);
        output_count[field_idx] = std::to_string(count);
        if(state == 1) // 1 = active = yellow
        {
            output_state[field_idx] = "[ON]";
            output_colors[field_idx] = yellow();
        }
        else // 0 = not active = gray or 2 = not used = gray
        {
            output_state[field_idx] = "[OFF]";
            output_colors[field_idx] = gray();
        }
    }
    std::stringstream dbg_info;
    dbg_info << "SickScanMarker::updateMarker(): LIDoutputstateMsg (state,count) = { ";
    for(int field_idx = 0; field_idx < num_devices; field_idx++)
        dbg_info << ((field_idx > 0) ? ", (" : "(") << output_state[field_idx] << "," << output_count[field_idx] << ")";
    dbg_info << " }";
    ROS_DEBUG_STREAM(dbg_info.str());
    m_scan_fieldset_legend = createMonFieldsetLegend(m_scan_mon_fieldset);
    m_scan_outputstate_legend = createOutputStateLegend(output_state, output_count, output_colors);
    publishMarker();
}

void sick_scan::SickScanMarker::updateMarker(sick_scan::LFErecMsg& msg)
{
    SickScanFieldMonSingleton *fieldMon = SickScanFieldMonSingleton::getInstance();
    if(fieldMon)
    {
        m_scan_mon_fieldset = fieldMon->getActiveFieldset();
        ROS_DEBUG_STREAM("SickScanMarker: active_fieldset = " << fieldMon->getActiveFieldset());
    }
    std::vector<FieldInfo> field_info(msg.fields.size());
    for(int field_idx = 0; field_idx < msg.fields.size(); field_idx++)
    {
        // LFErec: field_index runs from 1 to 3, field_info: field_index_scan_mon runs from 0 to 47 (field_index of m_scan_mon_fields)
        field_info[field_idx].field_index_scan_mon = msg.fields[field_idx].field_index - 1 + msg.fields.size() * m_scan_mon_fieldset; 
        field_info[field_idx].field_result = msg.fields[field_idx].field_result_mrs;
        if(field_info[field_idx].field_result == 1) // 1 = free/clear = green
        {
            field_info[field_idx].field_status = "Clear";
            field_info[field_idx].field_color = green();
        }
        else if(field_info[field_idx].field_result == 2) // 2 = infringed = yellow
        {
            field_info[field_idx].field_status = "Infringed";
            field_info[field_idx].field_color = yellow();
        }
        else // 0 = invalid = gray
        {
            field_info[field_idx].field_status = "Incorrect";
            field_info[field_idx].field_color = gray();
        }
        field_info[field_idx].field_name = std::to_string(field_info.size() - field_idx); // field_info[field_info_idx].field_index;

    }
    std::stringstream dbg_info;
    dbg_info << "SickScanMarker::updateMarker(): LFErec states={";
    for(int field_idx = 0; field_idx < msg.fields.size(); field_idx++)
        dbg_info << ((field_idx > 0) ? "," : "") << (int)msg.fields[field_idx].field_index << ":" << (int)msg.fields[field_idx].field_result_mrs;
    dbg_info << "}, mon_field_point_cnt={";
    for(int field_idx = 0; field_idx < m_scan_mon_fields.size(); field_idx++)
        dbg_info << ((field_idx > 0) ? "," : "") << m_scan_mon_fields[field_idx].getPointCount();
    dbg_info << "}, mon_field_set = " << m_scan_mon_fieldset;
    ROS_DEBUG_STREAM(dbg_info.str());
    m_scan_mon_field_marker = createMonFieldMarker(field_info);
    m_scan_mon_field_legend = createMonFieldLegend(field_info);
    m_scan_fieldset_legend = createMonFieldsetLegend(m_scan_mon_fieldset);
    publishMarker();
}

void sick_scan::SickScanMarker::publishMarker(void)
{
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.reserve(m_scan_mon_field_marker.size() + m_scan_mon_field_legend.size() + m_scan_outputstate_legend.size());
    for(int n = 0; n < m_scan_mon_field_marker.size(); n++)
        marker_array.markers.push_back(m_scan_mon_field_marker[n]);
    for(int n = 0; n < m_scan_mon_field_legend.size(); n++)
        marker_array.markers.push_back(m_scan_mon_field_legend[n]);
    for(int n = 0; n < m_scan_outputstate_legend.size(); n++)
        marker_array.markers.push_back(m_scan_outputstate_legend[n]);
    for(int n = 0; n < m_scan_fieldset_legend.size(); n++)
        marker_array.markers.push_back(m_scan_fieldset_legend[n]);
    m_marker_publisher.publish(marker_array);
}

std::vector<visualization_msgs::Marker> sick_scan::SickScanMarker::createMonFieldMarker(const std::vector<FieldInfo>& field_info)
{
    int nr_triangles = 0;
    for(int field_info_idx = 0; field_info_idx < field_info.size(); field_info_idx++)
    {
        int field_idx = field_info[field_info_idx].field_index_scan_mon;
        nr_triangles += std::max(0, m_scan_mon_fields[field_idx].getPointCount() - 1); // 2 points: 1 triangle, 3 points: 2 triangles, and so on
    }

    // Draw fields using marker triangles
    visualization_msgs::Marker marker_point;
    marker_point.header.stamp = ros::Time::now();
    marker_point.header.frame_id = m_frame_id;
    marker_point.ns = "sick_scan";
    marker_point.id = 1;
    marker_point.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker_point.scale.x = 1;
    marker_point.scale.y = 1;
    marker_point.scale.z = 1;
    marker_point.pose.position.x = 0.0;
    marker_point.pose.position.y = 0.0;
    marker_point.pose.position.z = 0.0;
    marker_point.pose.orientation.x = 0.0;
    marker_point.pose.orientation.y = 0.0;
    marker_point.pose.orientation.z = 0.0;
    marker_point.pose.orientation.w = 1.0;
    marker_point.action = visualization_msgs::Marker::ADD; // note: ADD == MODIFY
    marker_point.color = gray();
    marker_point.lifetime = ros::Duration(0); // lifetime 0 indicates forever

    marker_point.points.resize(3 * nr_triangles);
    marker_point.colors.resize(3 * nr_triangles);
    std::vector<geometry_msgs::Point> triangle_centroids;
    triangle_centroids.reserve(field_info.size());
    for(int field_info_idx = 0, triangle_idx = 0; field_info_idx < field_info.size() && triangle_idx < nr_triangles; field_info_idx++)
    {
        int field_idx = field_info[field_info_idx].field_index_scan_mon;
        std_msgs::ColorRGBA field_color = field_info[field_info_idx].field_color;
        for(int point_idx = 1; point_idx < m_scan_mon_fields[field_idx].getPointCount() && triangle_idx < nr_triangles; point_idx++, triangle_idx++)
        {
            float p1_x = 0, p1_y = 0, p1_z = 0, p2_x = 0, p2_y = 0, p2_z = 0, p3_x = 0, p3_y = 0, p3_z = 0;
            monFieldToCarthesian(m_scan_mon_fields[field_idx].getRanges()[point_idx - 1], m_scan_mon_fields[field_idx].getAnglesRad()[point_idx - 1], p2_x, p2_y);
            monFieldToCarthesian(m_scan_mon_fields[field_idx].getRanges()[point_idx - 0], m_scan_mon_fields[field_idx].getAnglesRad()[point_idx - 0], p3_x, p3_y);

            marker_point.points[3 * triangle_idx + 0].x = p1_x;
            marker_point.points[3 * triangle_idx + 0].y = p1_y;
            marker_point.points[3 * triangle_idx + 0].z = p1_z;

            marker_point.points[3 * triangle_idx + 1].x = p2_x;
            marker_point.points[3 * triangle_idx + 1].y = p2_y;
            marker_point.points[3 * triangle_idx + 1].z = p2_z;

            marker_point.points[3 * triangle_idx + 2].x = p3_x;
            marker_point.points[3 * triangle_idx + 2].y = p3_y;
            marker_point.points[3 * triangle_idx + 2].z = p3_z;

            marker_point.colors[3 * triangle_idx + 0] = field_color;
            marker_point.colors[3 * triangle_idx + 1] = field_color;
            marker_point.colors[3 * triangle_idx + 2] = field_color;
        }
    }

    std::vector<visualization_msgs::Marker> marker_array;
    marker_array.reserve(1 + field_info.size());
    marker_array.push_back(marker_point);

    // Draw field names
    for(int field_info_idx = 0; field_info_idx < field_info.size(); field_info_idx++)
    {
        int field_idx = field_info[field_info_idx].field_index_scan_mon;
        if(m_scan_mon_fields[field_idx].getPointCount() >= 2)
        {
            geometry_msgs::Point triangle_centroid;
            triangle_centroid.x = 0;
            triangle_centroid.y = 0;
            triangle_centroid.z = 0;
            for(int point_idx = 0; point_idx < m_scan_mon_fields[field_idx].getPointCount(); point_idx++)
            {
                float x = 0, y = 0, z = 0;
                monFieldToCarthesian(m_scan_mon_fields[field_idx].getRanges()[point_idx], m_scan_mon_fields[field_idx].getAnglesRad()[point_idx], x, y);
                triangle_centroid.x += x;
                triangle_centroid.y += y;
            }
            triangle_centroid.x /= (float)(m_scan_mon_fields[field_idx].getPointCount() + 1);
            triangle_centroid.y /= (float)(m_scan_mon_fields[field_idx].getPointCount() + 1);
            visualization_msgs::Marker marker_field_name;
            marker_field_name.header.stamp = ros::Time::now();
            marker_field_name.header.frame_id = m_frame_id;
            marker_field_name.ns = "sick_scan";
            marker_field_name.id = 2 + field_info_idx;
            marker_field_name.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker_field_name.scale.z = 0.1;
            marker_field_name.pose.position.x = triangle_centroid.x;
            marker_field_name.pose.position.y = triangle_centroid.y;
            marker_field_name.pose.position.z = triangle_centroid.z;
            marker_field_name.pose.orientation.x = 0.0;
            marker_field_name.pose.orientation.y = 0.0;
            marker_field_name.pose.orientation.z = 0.0;
            marker_field_name.pose.orientation.w = 1.0;
            marker_field_name.action = visualization_msgs::Marker::ADD; // note: ADD == MODIFY
            marker_field_name.color = field_info[field_info_idx].field_color;
            marker_field_name.color.a = 1;
            marker_field_name.lifetime = ros::Duration(0); // lifetime 0 indicates forever
            marker_field_name.text = field_info[field_info_idx].field_name;
            marker_array.push_back(marker_field_name);
        }
        else
        {
            visualization_msgs::Marker marker_field_name;
            marker_field_name.header.stamp = ros::Time::now();
            marker_field_name.header.frame_id = m_frame_id;
            marker_field_name.ns = "sick_scan";
            marker_field_name.id = 2 + field_info_idx;
            marker_field_name.action = visualization_msgs::Marker::DELETE;
            marker_field_name.lifetime = ros::Duration(0); // lifetime 0 indicates forever
            marker_array.push_back(marker_field_name);
        }
        
    }

    return marker_array;
}

std::vector<visualization_msgs::Marker> sick_scan::SickScanMarker::createMonFieldLegend(const std::vector<FieldInfo>& field_info)//, std_msgs::ColorRGBA default_color)
{
    std::vector<visualization_msgs::Marker> marker_array;
    marker_array.reserve(2 * field_info.size());
    for(int loop_cnt = 0; loop_cnt < 2; loop_cnt++)
    {
        for(int field_info_idx = 0, triangle_idx = 0; field_info_idx < field_info.size(); field_info_idx++)
        {
            int field_idx = field_info[field_info_idx].field_index_scan_mon;
            visualization_msgs::Marker marker_point;
            marker_point.header.stamp = ros::Time::now();
            marker_point.header.frame_id = m_frame_id;
            marker_point.ns = "sick_scan";
            marker_point.id = 100 + loop_cnt * m_scan_mon_fields.size() + field_info_idx;
            marker_point.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker_point.scale.z = 0.1;
            marker_point.pose.position.x = -0.1 * field_info_idx - 0.1;
            marker_point.pose.position.y = ((loop_cnt == 0) ? 0.3 : -0.2);
            marker_point.pose.position.z = 0.0;
            marker_point.pose.orientation.x = 0.0;
            marker_point.pose.orientation.y = 0.0;
            marker_point.pose.orientation.z = 0.0;
            marker_point.pose.orientation.w = 1.0;
            marker_point.action = visualization_msgs::Marker::ADD; // note: ADD == MODIFY
            marker_point.color = field_info[field_info_idx].field_color;
            marker_point.color.a = 1;
            marker_point.lifetime = ros::Duration(0); // lifetime 0 indicates forever
            std::stringstream marker_text;
            // int detection_field_number = field_info.size() - field_info_idx; // field_info[field_info_idx].field_index;
            if (loop_cnt == 0)
                marker_text << "Detection field " << (field_info[field_info_idx].field_name) << " : ";
            else
                marker_text << field_info[field_info_idx].field_status;
            marker_point.text = marker_text.str();
            marker_array.push_back(marker_point);
        }
    }
    return marker_array;
}

std::vector<visualization_msgs::Marker> sick_scan::SickScanMarker::createMonFieldsetLegend(int fieldset)
{
    std::vector<visualization_msgs::Marker> marker_array;
    marker_array.reserve(2);
    for(int loop_cnt = 0; loop_cnt < 2; loop_cnt++)
    {
        visualization_msgs::Marker marker_point;
        marker_point.header.stamp = ros::Time::now();
        marker_point.header.frame_id = m_frame_id;
        marker_point.ns = "sick_scan";
        marker_point.id = 500 + loop_cnt;
        marker_point.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker_point.scale.z = 0.1;
        marker_point.pose.position.x = -0.4;
        marker_point.pose.position.y = ((loop_cnt == 0) ? 0.16 : -0.2);
        marker_point.pose.position.z = 0.0;
        marker_point.pose.orientation.x = 0.0;
        marker_point.pose.orientation.y = 0.0;
        marker_point.pose.orientation.z = 0.0;
        marker_point.pose.orientation.w = 1.0;
        marker_point.action = visualization_msgs::Marker::ADD; // note: ADD == MODIFY
        marker_point.color = green();
        marker_point.color.a = 1;
        marker_point.lifetime = ros::Duration(0); // lifetime 0 indicates forever
        std::stringstream marker_text;
        if (loop_cnt == 0)
            marker_text << "Fieldset :";
        else
            marker_text << std::to_string(fieldset + 1);
        marker_point.text = marker_text.str();
        marker_array.push_back(marker_point);
    }
   return marker_array;
}


std::vector<visualization_msgs::Marker> sick_scan::SickScanMarker::createOutputStateLegend(const std::vector<std::string>& output_state, const std::vector<std::string>& output_count, const std::vector<std_msgs::ColorRGBA>& output_colors)
{
    std::vector<visualization_msgs::Marker> marker_array;
    marker_array.reserve(2 * output_count.size());
    for(int loop_cnt = 0; loop_cnt < 2; loop_cnt++)
    {
        for(int field_idx = 0; field_idx < output_count.size(); field_idx++)
        {
            visualization_msgs::Marker marker_point;
            marker_point.header.stamp = ros::Time::now();
            marker_point.header.frame_id = m_frame_id;
            marker_point.ns = "sick_scan";
            marker_point.id = 400 + loop_cnt * output_count.size() + field_idx;
            marker_point.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker_point.scale.z = 0.1;
            marker_point.pose.position.x = -0.1 * field_idx - 0.5;
            marker_point.pose.position.y = ((loop_cnt == 0) ? 0.16 : -0.3);
            marker_point.pose.position.z = 0.0;
            marker_point.pose.orientation.x = 0.0;
            marker_point.pose.orientation.y = 0.0;
            marker_point.pose.orientation.z = 0.0;
            marker_point.pose.orientation.w = 1.0;
            marker_point.action = visualization_msgs::Marker::ADD; // note: ADD == MODIFY
            marker_point.color = output_colors[field_idx];
            marker_point.color.a = 1;
            marker_point.lifetime = ros::Duration(0); // lifetime 0 indicates forever
            std::stringstream marker_text;
            int output_device = field_idx + 1;
            if (loop_cnt == 0)
                marker_text << "Output " << output_device << " : ";
            else
                marker_text << (field_idx < output_state.size() ? (output_state[field_idx]) : "") << " Count:" << output_count[field_idx];
            marker_point.text = marker_text.str();
            marker_array.push_back(marker_point);
        }
    }
    return marker_array;
}
