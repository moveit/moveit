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

#include <moveit_manipulation_visualization/object_recognition_dialog.h>

#include <QVBoxLayout>
#include <QPushButton>
#include <float.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <planning_models/transforms.h>

namespace moveit_manipulation_visualization {

ObjectRecognitionDialog::ObjectRecognitionDialog(QWidget* parent) :
  QDialog(parent)
{
  QVBoxLayout* layout = new QVBoxLayout(this);
  QPushButton* segment_button = new QPushButton(this);
  segment_button->setText("Segment");
  layout->addWidget(segment_button);
  
  connect(segment_button, SIGNAL(clicked()),
          this, SLOT(segmentTableAndClusters()));

  recognize_button_ = new QPushButton(this);
  recognize_button_->setText("Recognize");
  recognize_button_->setDisabled(true);

  layout->addWidget(recognize_button_);

  connect(recognize_button_, SIGNAL(clicked()),
          this, SLOT(doRecognition()));

}

void ObjectRecognitionDialog::removeLastSegmentation() {
  for(unsigned int i = 0; i < last_clusters_.size(); i++) {
    std::stringstream ss;
    ss << "cluster_" << i;
    requestObjectDelete(ss.str());
  }

  last_table_ = moveit_manipulation_msgs::Table();
  last_clusters_.clear();
}

void ObjectRecognitionDialog::segmentTableAndClusters() {
  removeLastSegmentation();
  recognize_button_->setDisabled(true);
  Q_EMIT segmentationRequested();
}

void ObjectRecognitionDialog::doRecognition() {
  Q_EMIT recognitionRequested(last_table_,last_clusters_);
}
void ObjectRecognitionDialog::gotTableAndClusters(moveit_manipulation_msgs::Table table,
                                                  std::vector<sensor_msgs::PointCloud> clusters)
{
  Eigen::Affine3d table_mat;
  planning_models::poseFromMsg(table.pose.pose, table_mat);

  Eigen::Affine3d table_trans(Eigen::Translation3d((table.x_min+table.x_max)/2.0, (table.y_min+table.y_max)/2.0, -.05)*Eigen::Quaterniond::Identity());

  moveit_msgs::CollisionObject coll;
  coll.id = "table";
  ROS_INFO_STREAM("Table frame is " << table.pose.header.frame_id);
  coll.header.frame_id = table.pose.header.frame_id;
  coll.primitive_poses.resize(1);
  planning_models::msgFromPose(table_mat*table_trans, coll.primitive_poses[0]);
  coll.primitives.resize(1);
  coll.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  coll.primitives[0].dimensions[0] = fabs(table.x_max-table.x_min);
  coll.primitives[0].dimensions[1] = fabs(table.y_max-table.y_min);
  coll.primitives[0].dimensions[2] = .1;
  QColor col(128, 128, 128, 255);
  Q_EMIT addCollisionObjectRequested(coll, col);

  last_table_ = table;
  last_clusters_ = clusters;

  ROS_INFO_STREAM("Got " << clusters.size() << " clusters");
  // sensor_msgs::PointCloud2 pc2;
  // sensor_msgs::convertPointCloudToPointCloud2 (clusters[0], pc2);

  if(clusters.size() > 0) {
    recognize_button_->setEnabled(true);
  }

  // pcl::io::savePCDFile("object.pcd", pc2);
  for(unsigned int i = 0; i < clusters.size(); i++) {
    std::stringstream ss;
    ss << "cluster_" << i;
    moveit_msgs::CollisionObject cluster_obj;
    cluster_obj.id = ss.str();
    cluster_obj.header.frame_id = clusters[i].header.frame_id;
    cluster_obj.primitives.resize(clusters[i].points.size());
    cluster_obj.primitive_poses.resize(clusters[i].points.size());
    for(unsigned int j = 0; j < clusters[i].points.size(); j++) {
      cluster_obj.primitives[j].type = shape_msgs::SolidPrimitive::BOX;
      cluster_obj.primitives[j].dimensions[0] = .01;
      cluster_obj.primitives[j].dimensions[1] = .01;
      cluster_obj.primitives[j].dimensions[2] = .01;
      cluster_obj.primitive_poses[j].orientation.w = 1.0;
      cluster_obj.primitive_poses[j].position.x = clusters[i].points[j].x;
      cluster_obj.primitive_poses[j].position.y = clusters[i].points[j].y;
      cluster_obj.primitive_poses[j].position.z = clusters[i].points[j].z;
    }
    ROS_INFO_STREAM("Point cloud size " << clusters[i].points.size());
    // for(unsigned int j = 0; j < clusters[i].points.size(); j++) {
    //   double xval = clusters[i].points[j].x;
    //   double yval = clusters[i].points[j].y;
    //   double zval = clusters[i].points[j].z;
    
    // cluster_obj.shapes.resize(1);
    // cluster_obj.shapes[0].type = shape_msgs::Shape::BOX;
    // cluster_obj.shapes[0].dimensions.resize(3);
    // cluster_obj.poses.resize(1);
    // double x = 0.0, y = 0.0, z = 0.0;
    // double xmin = DBL_MAX, ymin = DBL_MAX, zmin = DBL_MAX;
    // double xmax = -DBL_MAX, ymax = -DBL_MAX, zmax = -DBL_MAX;
    // for(unsigned int j = 0; j < clusters[i].points.size(); j++) {
    //   double xval = clusters[i].points[j].x;
    //   double yval = clusters[i].points[j].y;
    //   double zval = clusters[i].points[j].z;
    //   x += xval;
    //   y += yval;
    //   z += zval;
    //   if(xval < xmin) {
    //     xmin = xval; 
    //   }
    //   if(xval > xmax) {
    //     xmax = xval; 
    //   }
      
    //   if(yval < ymin) {
    //     ymin = yval; 
    //   }
    //   if(yval > ymax) {
    //     ymax = yval; 
    //   }
      
    //   if(zval < zmin) {
    //     zmin = zval; 
    //   }
    //   if(zval > zmax) {
    //     zmax = zval; 
    //   }
    // }
    // cluster_obj.shapes[0].dimensions[0] = fabs(xmax-xmin);
    // cluster_obj.shapes[0].dimensions[1] = fabs(ymax-ymin);
    // cluster_obj.shapes[0].dimensions[2] = fabs(zmax-zmin);
    // cluster_obj.poses[0].position.x = (x/(clusters[i].points.size()*1.0));
    // cluster_obj.poses[0].position.y = (y/(clusters[i].points.size()*1.0));
    // cluster_obj.poses[0].position.z = (z/(clusters[i].points.size()*1.0));
    //ROS_INFO_STREAM("Size " << fabs(xmax-xmin) << " " << fabs(ymax-ymin) << " " << fabs(zmax-zmin));
    //ROS_INFO_STREAM("Position " <<  cluster_obj.poses[0].position.x << " " << cluster_obj.poses[0].position.y << " " << cluster_obj.poses[0].position.z);
    //cluster_obj.poses[0].orientation.w = 1.0;
    ROS_INFO_STREAM("Emitting");
    Q_EMIT addCollisionObjectRequested(cluster_obj, col);
  }
}

void ObjectRecognitionDialog::gotObjectRecognition(std::vector<moveit_manipulation_msgs::DatabaseModelPoseList> model_lists,
                                                   std::vector<int> cluster_model_indices)
{
  //for(unsigned int i = 0; i < models.size(); i++) {
  //for(unsigned int j = 0; j < models[i].model_list.size(); j++) {
  //ROS_INFO_STREAM("For cluster " << i << " got model id " << models[i].model_list[j]);
  //}
  //}
  removeLastSegmentation();
  last_recognition_ = model_lists;
  last_cluster_model_indices_ = cluster_model_indices;
  
  for(unsigned int i = 0; i < model_lists.size(); i++) {
    if(model_lists[i].model_list.size() > 0) {
      std::stringstream ss;
      ss << "recognized_" << i << "_model_"<< model_lists[i].model_list[0].model_id;
      Q_EMIT requestHouseholdObjectAddition(ss.str(),
                                            model_lists[i].model_list[0].model_id,
                                            model_lists[i].model_list[0].pose.pose);
    }
  }
}

}
