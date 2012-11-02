/*
 * Copyright (c) 2012, Willow Garage, Inc.
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
 *     * Neither the name of the <ORGANIZATION> nor the names of its
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
 */

// Author: E. Gil Jones

#ifndef _OBJECT_RECOGNITION_QT_SERVICE_WRAPPER_H_
#define _OBJECT_RECOGNITION_QT_SERVICE_WRAPPER_H_

#include <ros/ros.h>
#include <QObject>

#include <moveit_manipulation_msgs/Table.h>
#include <sensor_msgs/PointCloud.h>
#include <moveit_manipulation_msgs/DatabaseModelPoseList.h>

namespace moveit_manipulation_visualization
{

class ObjectRecognitionQtServiceWrapper : public QObject
{
  Q_OBJECT

  public:

  ObjectRecognitionQtServiceWrapper();

public Q_SLOTS:

  void segmentAndCluster();

  void recognize(moveit_manipulation_msgs::Table,
                 std::vector<sensor_msgs::PointCloud>);
  
Q_SIGNALS:
  
  void tableAndClustersGenerated(moveit_manipulation_msgs::Table,
                                 std::vector<sensor_msgs::PointCloud>);

  void objectsRecognized(std::vector<moveit_manipulation_msgs::DatabaseModelPoseList>,
                         std::vector<int>);

protected:

  bool connectToSegmentationService();
  bool connectToRecognitionService();
  
  ros::ServiceClient seg_srv_;
  ros::ServiceClient rec_srv_;

};

}

#endif
