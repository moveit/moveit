/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/** \author Ioan Sucan */

#include <cstdio>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <moveit/robot_self_filter/self_mask.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class TestSelfFilter
{
public:

  TestSelfFilter()
  {
    id_ = 1;
    vmPub_ = nodeHandle_.advertise<visualization_msgs::Marker>("visualization_marker", 10240);

    ros::NodeHandle priv_nh("~");

    //nodeHandle_.param<double> ("min_sensor_dist", min_sensor_dist_, 0.01);
    double default_padding, default_scale;
    default_padding = 0.01;
    default_scale = 1.0;
    //nodeHandle_.param<double> ("self_see_default_padding", default_padding, .01);
    //nodeHandle_.param<double> ("self_see_default_scale", default_scale, 1.0);

    std::vector<robot_self_filter::LinkInfo> links;
    std::string link_names;

    if (!priv_nh.hasParam ("self_see_links"))
    {
      ROS_WARN ("No links specified for self filtering.");
    }
    else
    {
      XmlRpc::XmlRpcValue ssl_vals;;

      priv_nh.getParam ("self_see_links", ssl_vals);
      if (ssl_vals.getType () != XmlRpc::XmlRpcValue::TypeArray)
      {
        ROS_WARN ("Self see links need to be an array");
      }
      else
      {
        if (ssl_vals.size () == 0)
        {
          ROS_WARN ("No values in self see links array");
        }
        else
        {
          for (int i = 0; i < ssl_vals.size (); ++i)
          {
            robot_self_filter::LinkInfo li;

            if (ssl_vals[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
            {
              ROS_WARN ("Self see links entry %d is not a structure.  Stopping processing of self see links", i);
              break;
            }
            if (!ssl_vals[i].hasMember ("name"))
            {
              ROS_WARN ("Self see links entry %d has no name.  Stopping processing of self see links", i);
              break;
            }
            li.name = std::string (ssl_vals[i]["name"]);
            if (!ssl_vals[i].hasMember ("padding"))
            {
              ROS_DEBUG ("Self see links entry %d has no padding.  Assuming default padding of %g", i, default_padding);
              li.padding = default_padding;
            }
            else
            {
              li.padding = ssl_vals[i]["padding"];
            }
            if (!ssl_vals[i].hasMember ("scale"))
            {
              ROS_DEBUG ("Self see links entry %d has no scale.  Assuming default scale of %g", i, default_scale);
              li.scale = default_scale;
            }
            else
            {
              li.scale = ssl_vals[i]["scale"];
            }
            links.push_back (li);
          }
        }
      }
    }

    sf_ = new robot_self_filter::SelfMask (tf_, links);

  }

  ~TestSelfFilter()
  {
    delete sf_;
  }

  void gotIntersection(const Eigen::Vector3d &pt)
  {
    sendPoint(pt.x(), pt.y(), pt.z());
  }

  void sendPoint(double x, double y, double z)
  {
    visualization_msgs::Marker mk;

    mk.header.stamp = ros::Time(0);

    mk.header.frame_id = "/base_link";

    mk.ns = "test_self_filter";
    mk.id = id_++;
    mk.type = visualization_msgs::Marker::SPHERE;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.position.x = x;
    mk.pose.position.y = y;
    mk.pose.position.z = z;
    mk.pose.orientation.w = 1.0;

    mk.scale.x = mk.scale.y = mk.scale.z = 0.01;

    mk.color.a = 0.5;
    mk.color.r = 1.0;
    mk.color.g = 0.04;
    mk.color.b = 0.04;

   // mk.lifetime = ros::Duration(10);

    vmPub_.publish(mk);
  }

  void run()
  {
    pcl::PointCloud<pcl::PointXYZ> in;

    in.header.stamp = ros::Time(0);
    in.header.frame_id = "/base_link";
    /*	in.channels.resize(1);
  in.channels[0].name = "stamps";*/

    const unsigned int N = 100000;
    in.points.resize(N);
    /*	in.channels[0].values.resize(N);*/
    for (unsigned int i = 0 ; i < N ; ++i)
    {
      in.points[i].x = uniform(1.5);
      in.points[i].y = uniform(1.5);
      in.points[i].z = uniform(1.5);
      /*	    in.channels[0].values[i] = (double)i/(double)N;*/
    }

    for (unsigned int i = 0 ; i < 1000 ; ++i)
    {
      ros::Duration(0.01).sleep();
      ros::spinOnce();
    }

    ros::WallTime tm = ros::WallTime::now();
    std::vector<int> mask;
    //sf_->maskIntersection(in, "", 0.01, mask, boost::bind(&TestSelfFilter::gotIntersection, this, _1) );
    sf_->maskContainment(in, mask);
    //	sf_->maskContainment(in, mask);
    printf("%f points per second\n", (double)N/(ros::WallTime::now() - tm).toSec());


    int k = 0;
    for (unsigned int i = 0 ; i < mask.size() ; ++i)
    {
      //	    bool v = sf_->getMaskContainment(in.points[i].x, in.points[i].y, in.points[i].z);
      //	    if (v != mask[i])
      //		ROS_ERROR("Mask does not match");
      if (mask[i] == robot_self_filter::INSIDE)
            sendPoint(in.points[i].x, in.points[i].y, in.points[i].z);
      k++;
    }

    ros::spin();
  }

protected:

  double uniform(double magnitude)
  {
    return (2.0 * drand48() - 1.0) * magnitude;
  }

  tf::TransformListener             tf_;
  robot_self_filter::SelfMask      *sf_;
  ros::Publisher                    vmPub_;
  ros::NodeHandle                   nodeHandle_;
  int                               id_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_self_filter");

  TestSelfFilter t;
  sleep(1);
  t.run();

  return 0;
}



