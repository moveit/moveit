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

#ifndef ROBOT_SELF_FILTER_SELF_MASK_
#define ROBOT_SELF_FILTER_SELF_MASK_

#include <sensor_msgs/PointCloud.h>
#include <geometric_shapes/bodies.h>
#include <tf/transform_listener.h>
#include <boost/bind.hpp>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace robot_self_filter
{
    /** \brief The possible values of a mask computed for a point */
    enum
    {
      INSIDE = 0,
      OUTSIDE = 1,
      SHADOW = 2,
    };

    struct LinkInfo
    {
      std::string name;
      double padding;
      double scale;
    };
        
    /** \brief Computing a mask for a pointcloud that states which points are inside the robot */
    class SelfMask
    {	
      protected:
    
        struct SeeLink
        {
          SeeLink(void)
          {
            body = unscaledBody = NULL;
          }
            
          std::string   name;
          bodies::Body *body;
          bodies::Body *unscaledBody;
          tf::Transform   constTransf;
          double        volume;
        };
        
        struct SortBodies
        {
          bool operator()(const SeeLink &b1, const SeeLink &b2)
          {
            return (b1.volume > b2.volume);
          }
        };
        
      public:
      
        /** \brief Construct the filter */
        SelfMask(tf::TransformListener &tf, const std::vector<LinkInfo> &links) : tf_(tf)
        {
          configure (links);
        }
        
        /** \brief Destructor to clean up */
        ~SelfMask(void)
        {
          freeMemory ();
        }
        
        /** \brief Compute the containment mask (INSIDE or OUTSIDE) for a given pointcloud. If a mask element is INSIDE, the point
            is inside the robot. The point is outside if the mask element is OUTSIDE.
         */
        void maskContainment (const pcl::PointCloud<pcl::PointXYZ>& data_in, std::vector<int> &mask);

        /** \brief Compute the intersection mask for a given
            pointcloud. If a mask element can have one of the values
            INSIDE, OUTSIDE or SHADOW. If the value is SHADOW, the
            point is on a ray behind the robot and should not have
            been seen. If the mask element is INSIDE, the point is
            inside the robot. The sensor frame is specified to obtain
            the origin of the sensor. A callback can be registered for
            the first intersection point on each body.
         */
        void maskIntersection (const pcl::PointCloud<pcl::PointXYZ>& data_in, const std::string &sensor_frame, const double min_sensor_dist,
                  std::vector<int> &mask, const boost::function<void(const tf::Vector3&)> &intersectionCallback = NULL);

        /** \brief Compute the intersection mask for a given pointcloud. If a mask
            element can have one of the values INSIDE, OUTSIDE or SHADOW. If the value is SHADOW,
            the point is on a ray behind the robot and should not have
            been seen. If the mask element is INSIDE, the point is inside
            the robot. The origin of the sensor is specified as well.
         */
        void maskIntersection (const pcl::PointCloud<pcl::PointXYZ>& data_in, const tf::Vector3 &sensor, const double min_sensor_dist,
                  std::vector<int> &mask, const boost::function<void(const tf::Vector3&)> &intersectionCallback = NULL);
        
        /** \brief Assume subsequent calls to getMaskX() will be in the frame passed to this function.
         *   The frame in which the sensor is located is optional */
        void assumeFrame (const std::string &frame_id, const ros::Time &stamp);
        
        /** \brief Assume subsequent calls to getMaskX() will be in the frame passed to this function.
         *   The frame in which the sensor is located is optional */
        void assumeFrame (const std::string &frame_id, const ros::Time &stamp, const std::string &sensor_frame, const double min_sensor_dist);

        /** \brief Assume subsequent calls to getMaskX() will be in the frame passed to this function.
         *  Also specify which possition to assume for the sensor (frame is not needed) */
        void assumeFrame (const std::string &frame_id, const ros::Time &stamp, const tf::Vector3 &sensor_pos, const double min_sensor_dist);
        
        /** \brief Get the containment mask (INSIDE or OUTSIDE) value for an individual point. No
            setup is performed, assumeFrame() should be called before use */
        int getMaskContainment (double x, double y, double z) const;
        
        /** \brief Get the containment mask (INSIDE or OUTSIDE) value for an individual point. No
            setup is performed, assumeFrame() should be called before use */
        int getMaskContainment (const tf::Vector3 &pt) const;
        
        /** \brief Get the intersection mask (INSIDE, OUTSIDE or
            SHADOW) value for an individual point. No setup is
            performed, assumeFrame() should be called before use */
        int getMaskIntersection (double x, double y, double z, const boost::function<void(const tf::Vector3&)> &intersectionCallback = NULL) const;
        
        /** \brief Get the intersection mask (INSIDE, OUTSIDE or
            SHADOW) value for an individual point. No setup is
            performed, assumeFrame() should be called before use */
        int getMaskIntersection (const tf::Vector3 &pt, const boost::function<void(const tf::Vector3&)> &intersectionCallback = NULL) const;
        
        /** \brief Get the set of link names that have been instantiated for self filtering */
        void getLinkNames (std::vector<std::string> &frames) const;
      
      private:
        /** \brief Free memory. */
        void freeMemory (void);

        /** \brief Configure the filter. */
        bool configure (const std::vector<LinkInfo> &links);
        
        /** \brief Compute bounding spheres for the checked robot links. */
        void computeBoundingSpheres (void);
        
        /** \brief Perform the actual mask computation. */
        void maskAuxContainment (const pcl::PointCloud<pcl::PointXYZ>& data_in, std::vector<int> &mask);

        /** \brief Perform the actual mask computation. */
        void maskAuxIntersection (const pcl::PointCloud<pcl::PointXYZ>& data_in, std::vector<int> &mask, const boost::function<void(const tf::Vector3&)> &callback);
        
        tf::TransformListener               &tf_;
        ros::NodeHandle                     nh_;
        
        tf::Vector3                           sensor_pos_;
        double                              min_sensor_dist_;
        
        std::vector<SeeLink>                bodies_;
        std::vector<double>                 bspheresRadius2_;
        std::vector<bodies::BoundingSphere> bspheres_;
    };
}

#endif
