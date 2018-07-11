/*
 * Copyright (C) 2018, SICK AG
 * Copyright (C) 2018, Ing.-Buero Dr. Michael Lehning
 * Copyright (C) 2016, DFKI GmbH
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
 *     * Neither the name of DFKI GmbH nor the names of its
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
 */

#include <ros/ros.h>

#include <visualization_msgs/MarkerArray.h>
#include <sick_scan/RadarScan.h>

#include <ros/ros.h>

#include <visualization_msgs/MarkerArray.h>
#include <sick_scan/RadarScan.h>
#include "radar_object_marker/radar_object_marker.h"
float ballRadius = 0.1;


ros::Publisher pub;

void callback(const sick_scan::RadarScan::ConstPtr &oa)
  {
  enum TARGET_MARKER_TYPE {TARGET_MARKER_ARROW, TARGET_MARKER_NUM, TARGET_MARKER_BALL};
  visualization_msgs::MarkerArray rawTargetArray[TARGET_MARKER_NUM];  // ball and arrow

  for (int i = 0; i < TARGET_MARKER_NUM; i++)
  {
    rawTargetArray[i].markers.resize(oa->targets.width);
  }

  int coordIdx[3] = {0};
  int vradIdx = -1;
  int amplitudeIdx = -1;
  for (int i = 0; i < 3; i++)
  {
    coordIdx[i] = -1;
  }

  for (int i = 0; i < oa->targets.fields.size(); i++)
  {
    std::string fieldName = oa->targets.fields[i].name;
    // x,y,z, vrad, amplitude
    if (fieldName.compare("x") == 0)
    {
      coordIdx[0] = i;
    }
    if (fieldName.compare("y") == 0)
    {
      coordIdx[1] = i;
    }
    if (fieldName.compare("z") == 0)
    {
      coordIdx[2] = i;
    }
    if (fieldName.compare("vrad") == 0)
    {
      vradIdx = i;
    }

    // printf("%s\n", fieldName.c_str());
  }

  int numRawTargets = oa->targets.width;
  for (size_t i = 0; i < oa->targets.width; i++)
  {
    int tmpId = i;
    float xs, ys;
    float pts3d[3];
    float vrad = 0.0;
    for (int ii = 0; ii < 3; ii++)
    {
      if (coordIdx[ii] != -1)
      {
        int tmpIdx = coordIdx[ii];
        float *valPtr = (float *) (&(oa->targets.data[0]) + i * oa->targets.point_step +
                                   oa->targets.fields[tmpIdx].offset);
        pts3d[ii] = *valPtr;
      }
    }
    if (vradIdx != -1)
    {
      int tmpIdx = vradIdx;
      float *valPtr = (float *) (&(oa->targets.data[0]) + i * oa->targets.point_step +
                                 oa->targets.fields[tmpIdx].offset);
      vrad = *valPtr;
    }

    for (int iLoop = 0; iLoop < TARGET_MARKER_NUM; iLoop++)
    {

      rawTargetArray[iLoop].markers[i].header = oa->header;
      rawTargetArray[iLoop].markers[i].ns = "rawtargets";
      rawTargetArray[iLoop].markers[i].id = tmpId + iLoop * numRawTargets;
      switch (iLoop)
      {
        case TARGET_MARKER_ARROW:
          rawTargetArray[iLoop].markers[i].type = visualization_msgs::Marker::ARROW;
          rawTargetArray[iLoop].markers[i].scale.x = 0.1;
          rawTargetArray[iLoop].markers[i].scale.y = 0.2;
          break;
        case TARGET_MARKER_BALL:
          rawTargetArray[iLoop].markers[i].type = visualization_msgs::Marker::SPHERE;
          rawTargetArray[iLoop].markers[i].scale.x = ballRadius;
          rawTargetArray[iLoop].markers[i].scale.y = ballRadius;
          break;

      }
      rawTargetArray[iLoop].markers[i].action = visualization_msgs::Marker::ADD;
      rawTargetArray[iLoop].markers[i].color.a = 0.75;
      rawTargetArray[iLoop].markers[i].color.r = GLASBEY_LUT[tmpId * 3] / 255.0;
      rawTargetArray[iLoop].markers[i].color.g = GLASBEY_LUT[tmpId * 3 + 1] / 255.0;
      rawTargetArray[iLoop].markers[i].color.b = GLASBEY_LUT[tmpId * 3 + 2] / 255.0;
      rawTargetArray[iLoop].markers[i].lifetime = ros::Duration(0.5);

      rawTargetArray[iLoop].markers[i].points.resize(2);
      rawTargetArray[iLoop].markers[i].points[0].x = pts3d[0];
      rawTargetArray[iLoop].markers[i].points[0].y = pts3d[1];
      rawTargetArray[iLoop].markers[i].points[0].z = pts3d[2];
      float lineOfSight = atan2(pts3d[1], pts3d[0]);
      rawTargetArray[iLoop].markers[i].points[1].x = pts3d[0] + cos(lineOfSight) * vrad * 0.1;
      rawTargetArray[iLoop].markers[i].points[1].y = pts3d[1] + sin(lineOfSight) * vrad * 0.1;
      rawTargetArray[iLoop].markers[i].points[1].z = pts3d[2];

    }
  }

  for (int iLoop = 0; iLoop < TARGET_MARKER_NUM; iLoop++)
  {
    pub.publish(rawTargetArray[iLoop]);
  }
#if 1
  visualization_msgs::MarkerArray object_boxes;
  object_boxes.markers.resize(2 * oa->objects.size());

  for (int iLoop = 0; iLoop < 2; iLoop++)
  {
    for (size_t i = 0; i < oa->objects.size(); i++)
    {
      int idx = i + iLoop * oa->objects.size();
      int tmpId = i; // better: oa->objects[i].id;
      object_boxes.markers[idx].header = oa->header;
      object_boxes.markers[idx].ns = "object_boxes";
      object_boxes.markers[idx].id = tmpId + iLoop * oa->objects.size(); // i; // oa->objects[i].id;
      object_boxes.markers[idx].action = visualization_msgs::Marker::ADD;
      object_boxes.markers[idx].color.a = 0.75;
      object_boxes.markers[idx].color.r = GLASBEY_LUT[tmpId * 3] / 255.0;
      object_boxes.markers[idx].color.g = GLASBEY_LUT[tmpId * 3 + 1] / 255.0;
      object_boxes.markers[idx].color.b = GLASBEY_LUT[tmpId * 3 + 2] / 255.0;
      object_boxes.markers[idx].lifetime = ros::Duration(2.5);

      object_boxes.markers[idx].pose = oa->objects[i].object_box_center.pose;
      object_boxes.markers[idx].scale = oa->objects[i].object_box_size;
      switch (iLoop)
      {
        case 0:
          object_boxes.markers[idx].type = visualization_msgs::Marker::CUBE;
          object_boxes.markers[idx].pose.position.z  +=  oa->objects[i].object_box_size.z * 0.5;
          break;
        case 1:
          object_boxes.markers[idx].type = visualization_msgs::Marker::ARROW;
          break;
      }

      switch (iLoop)
      {
        case 0:
          if (object_boxes.markers[idx].scale.x == 0.0)
          {
            object_boxes.markers[idx].scale.x = 0.01;
          }
          if (object_boxes.markers[idx].scale.y == 0.0)
          {
            object_boxes.markers[idx].scale.y = 0.01;
          }
          if (object_boxes.markers[idx].scale.z == 0.0)
          {
            object_boxes.markers[idx].scale.z = 0.01;
          }
          break;
        case 1:
        {
          float xTmp;
          float yTmp;

          object_boxes.markers[idx].scale.x = 0.5;
          object_boxes.markers[idx].scale.y = 0.5;
          object_boxes.markers[idx].scale.z = 0.5;

          float cosVal =  oa->objects[i].object_box_center.pose.orientation.x;
          float sinVal =  oa->objects[i].object_box_center.pose.orientation.y;

          xTmp = oa->objects[i].object_box_center.pose.position.x;
          yTmp = oa->objects[i].object_box_center.pose.position.y;

          double vehLen = oa->objects[i].object_box_size.x;
          xTmp += cosVal * vehLen * 0.5;
          yTmp += sinVal * vehLen * 0.5;
          object_boxes.markers[idx].pose.position.x = xTmp;
          object_boxes.markers[idx].pose.position.y = yTmp;
          object_boxes.markers[idx].pose.position.z = 0.0;

          object_boxes.markers[idx].points.resize(2);
          object_boxes.markers[idx].points[0].x = 0.0;
          object_boxes.markers[idx].points[0].y = 0.0;

          object_boxes.markers[idx].points[1].x = 0.0 + 5.0 * cosVal;
          object_boxes.markers[idx].points[1].y = 0.0 + 5.0 * sinVal;
        }
          break;
      }

      // printf("Idx: %2d X: %8.3lf Y: %8.3lf\n", idx, object_boxes.markers[idx].pose.position.x, object_boxes.markers[idx].pose.position.y);
    }
  }
  pub.publish(object_boxes);
#endif
  }


  int main(int argc, char **argv)
    {
    ros::init(argc, argv, "sick_scan_radar_object_marker");
    ros::NodeHandle nh;
    ros::NodeHandle nhPriv("~");
    bool paramRes = nhPriv.getParam("rawtarget_sphere_radius", ballRadius);
    ros::Subscriber sub = nh.subscribe("radar", 1, callback);
    pub = nh.advertise<visualization_msgs::MarkerArray>("radar_markers", 1);

    ros::spin();

    return 0;
    }