/*
 * Copyright (c) 2008, Willow Garage, Inc.
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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

#ifndef FILTERS_SELF_SEE_H_
#define FILTERS_SELF_SEE_H_

#include <filters/filter_base.h>
#include <robot_self_filter/self_mask.h>
#include <ros/console.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace filters
{
  /** \brief A filter to remove parts of the robot seen in a pointcloud */
  template <typename T>
  class SelfFilter: public FilterBase <T>
  {
    public:
      /** \brief Construct the filter */
      SelfFilter (ros::NodeHandle nh) : nh_(nh)
      {
        nh_.param<double> ("min_sensor_dist", min_sensor_dist_, 0.01);
        double default_padding, default_scale;
        nh_.param<double> ("self_see_default_padding", default_padding, .01);
        nh_.param<double> ("self_see_default_scale", default_scale, 1.0);
        
        std::vector<robot_self_filter::LinkInfo> links;	
        std::string link_names;
        
        if (!nh_.hasParam ("self_see_links")) 
        {
          ROS_WARN ("No links specified for self filtering.");
        } 
        else 
        {     
          XmlRpc::XmlRpcValue ssl_vals;;
          
          nh_.getParam ("self_see_links", ssl_vals);
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
        sm_ = new robot_self_filter::SelfMask (tf_, links);
//        nh_.param<std::string> ("annotate", annotate_, std::string ());
//        if (!annotate_.empty ())
//          ROS_INFO ("Self filter is adding annotation channel '%s'", annotate_.c_str ());
        if (!sensor_frame_.empty ())
          ROS_INFO("Self filter is removing shadow points for sensor in frame '%s'. Minimum distance to sensor is %f.", sensor_frame_.c_str (), min_sensor_dist_);
      }
        
      /** \brief Destructor to clean up
       */
      virtual ~SelfFilter (void)
      {
        delete sm_;
      }
      
      virtual bool 
        configure (void)
      {
        // keep only the points that are outside of the robot
        // for testing purposes this may be changed to true
        nh_.param ("invert", invert_, false);
        
        if (invert_)
          ROS_INFO ("Inverting filter output");
      
        return (true);
      }

      bool updateWithSensorFrame
        (const pcl::PointCloud<pcl::PointXYZ>& data_in, pcl::PointCloud<pcl::PointXYZ>& data_out, const std::string& sensor_frame)
      {
        sensor_frame_ = sensor_frame;
        return (update (data_in, data_out));
      }
        
      /** \brief Update the filter and return the data seperately
       * \param data_in T array with length width
       * \param data_out T array with length width
       */
      virtual bool 
        update (const pcl::PointCloud<pcl::PointXYZ>& data_in, pcl::PointCloud<pcl::PointXYZ>& data_out)
      {
        std::vector<int> keep (data_in.points.size ());
        if (sensor_frame_.empty ()) 
        {
          sm_->maskContainment (data_in, keep);
        } 
        else 
        {
          sm_->maskIntersection (data_in, sensor_frame_, min_sensor_dist_, keep);
        }	
        fillResult (data_in, keep, data_out);
        return (true);
      }

      bool updateWithSensorFrame (const pcl::PointCloud<pcl::PointXYZ>& data_in, pcl::PointCloud<pcl::PointXYZ>& data_out, pcl::PointCloud<pcl::PointXYZ>& data_diff, const std::string& sensor_frame)
      {
        sensor_frame_ = sensor_frame;
        return (update (data_in, data_out, data_diff));
      }

      /** \brief Update the filter and return the data seperately
       * \param data_in T array with length width
       * \param data_out T array with length width
       */
      virtual bool update (const pcl::PointCloud<pcl::PointXYZ>& data_in, pcl::PointCloud<pcl::PointXYZ>& data_out, pcl::PointCloud<pcl::PointXYZ>& data_diff)
      {
        std::vector<int> keep (data_in.points.size ());
        if (sensor_frame_.empty ()) 
        {
          sm_->maskContainment (data_in, keep);
        } 
        else 
        {
          sm_->maskIntersection (data_in, sensor_frame_, min_sensor_dist_, keep);
        }
        fillResult (data_in, keep, data_out);
        fillDiff (data_in,keep,data_diff);
        return (true);
      }

      void fillDiff (const pcl::PointCloud<pcl::PointXYZ>& data_in, const std::vector<int> &keep, pcl::PointCloud<pcl::PointXYZ>& data_out)
      {
        const unsigned int np = data_in.points.size ();
      
        // fill in output data 
        data_out.header = data_in.header;	  
      
        data_out.points.resize (0);
        data_out.points.reserve (np);
      
/*        data_out.channels.resize(data_in.channels.size());
        for (unsigned int i = 0 ; i < data_out.channels.size () ; ++i)
        {
          ROS_ASSERT(data_in.channels[i].values.size() == data_in.points.size());
          data_out.channels[i].name = data_in.channels[i].name;
          data_out.channels[i].values.reserve(data_in.channels[i].values.size());
        }
  */    
        for (unsigned int i = 0 ; i < np ; ++i)
        {
          if ((keep[i] && invert_) || (!keep[i] && !invert_))
          {
            data_out.points.push_back(data_in.points[i]);
/*            for (unsigned int j = 0 ; j < data_out.channels.size() ; ++j)
              data_out.channels[j].values.push_back(data_in.channels[j].values[i]);*/
          }
        }
      }

      void fillResult (const pcl::PointCloud<pcl::PointXYZ>& data_in, const std::vector<int> &keep, pcl::PointCloud<pcl::PointXYZ>& data_out)
      {
        const unsigned int np = data_in.points.size ();

        // fill in output data with points that are NOT on the robot
        data_out.header = data_in.header;	  
      
        data_out.points.resize (0);
        data_out.points.reserve (np);
      
/*        data_out.channels.resize(data_in.channels.size());
        for (unsigned int i = 0 ; i < data_out.channels.size() ; ++i)
        {
          ROS_ASSERT(data_in.channels[i].values.size() == data_in.points.size());
          data_out.channels[i].name = data_in.channels[i].name;
          data_out.channels[i].values.reserve(data_in.channels[i].values.size());
        }*/
      
/*        int c = -1;
        if (!annotate_.empty())
        {
          // add annotation for points
          for (unsigned int i = 0 ; i < data_out.channels.size() ; ++i)
            if (data_out.channels[i].name == annotate_)
            {
              c = i;
              break;
            }
          if (c < 0)
          {
            c = data_out.channels.size();
            data_out.channels.resize(c + 1);
            data_out.channels[c].name = annotate_;
          }
          data_out.channels[c].values.reserve(np);
        }*/

        for (unsigned int i = 0 ; i < np ; ++i)
          //if (annotate_.empty())
          {
            if (keep[i] == robot_self_filter::OUTSIDE)
            {
              data_out.points.push_back (data_in.points[i]);
//              for (unsigned int j = 0 ; j < data_out.channels.size () ; ++j)
//                data_out.channels[j].values.push_back (data_in.channels[j].values[i]);
            }
          }
/*          else
          {
            data_out.points.push_back(data_in.points[i]);
            for (unsigned int j = 0 ; j < data_out.channels.size() ; ++j)
            {
              if ((int)j == c)
              {
                float flag = 0.0;
                if (keep[i] != robot_self_filter::SHADOW)
                  flag = keep[i] == robot_self_filter::OUTSIDE ? 1.0f : -1.0f;			
                data_out.channels[c].values.push_back(flag);
              }
              else
                data_out.channels[j].values.push_back(data_in.channels[j].values[i]);
            }
          }*/
      }

    /*  virtual bool updateWithSensorFrame(const std::vector<sensor_msgs::PointCloud> & data_in, std::vector<sensor_msgs::PointCloud>& data_out, const std::string& sensor_frame)
      {
        sensor_frame_ = sensor_frame;
        return update(data_in, data_out);
      }
      
      virtual bool update(const std::vector<sensor_msgs::PointCloud> & data_in, std::vector<sensor_msgs::PointCloud>& data_out)
      {
        bool result = true;
        data_out.resize(data_in.size());
        for (unsigned int i = 0 ; i < data_in.size() ; ++i)
          if (!update(data_in[i], data_out[i]))
            result = false;
        return true;
      }*/

      robot_self_filter::SelfMask* getSelfMask () 
      {
        return (sm_);
      }

      void setSensorFrame (const std::string& frame) 
      {
        sensor_frame_ = frame;
      }
        
    protected:
        
      tf::TransformListener tf_;
      robot_self_filter::SelfMask* sm_;
      
      ros::NodeHandle nh_;
      bool invert_;
      std::string sensor_frame_;
      std::string annotate_;
      double min_sensor_dist_;
  };
}

#endif //#ifndef FILTERS_SELF_SEE_H_
