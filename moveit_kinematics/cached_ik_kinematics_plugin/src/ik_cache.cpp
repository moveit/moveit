/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2017, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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

/* Author: Mark Moll */

#include <boost/filesystem/fstream.hpp>
#include <chrono>
#include <cstdlib>
#include <numeric>

#include <moveit/cached_ik_kinematics_plugin/cached_ik_kinematics_plugin.h>

namespace cached_ik_kinematics_plugin
{
    IKCache::IKCache()
    {
        // set distance function for nearest-neighbor queries
        ikNN_.setDistanceFunction([this](const IKEntry *entry1, const IKEntry *entry2)
                                  {
                                      double dist = 0.;
                                      for (unsigned int i = 0; i < entry1->first.size(); ++i)
                                          dist += entry1->first[i].distance(entry2->first[i]);
                                      return dist;
                                  });
    }

    IKCache::~IKCache()
    {
        if (!ikCache_.empty())
            saveCache();
    }

    void IKCache::initializeCache(
        const std::string &robot_description,
        const std::string &group_name,
        const std::string &cache_name,
        const unsigned int num_joints)
    {
        // read ROS parameters
        ros::NodeHandle node_handle("~");
        int maxCacheSize; // node_handle.param(...) doesn't handle unsigned ints
        node_handle.param(group_name+"/max_cache_size", maxCacheSize, 5000);
        maxCacheSize_ = maxCacheSize;
        ikCache_.reserve(maxCacheSize_);
        node_handle.param(group_name+"/min_pose_distance", minPoseDistance_, 1.);
        node_handle.param(group_name+"/min_joint_config_distance", minConfigDistance2_, 1.);
        minConfigDistance2_ *= minConfigDistance2_;
        std::string cached_ik_path;
        node_handle.param("cached_ik_path", cached_ik_path, std::string());

        // use mutex lock for rest of initialization
        std::lock_guard<std::mutex> slock(lock_);
        // determine cache file name
        boost::filesystem::path prefix(!cached_ik_path.empty() ? cached_ik_path : boost::filesystem::current_path());
        // create cache directory if necessary
        boost::filesystem::create_directories(prefix);

        cacheFileName_ = prefix / (robot_description + group_name + "_" + cache_name +
            "_" + std::to_string(maxCacheSize_) + "_" + std::to_string(minPoseDistance_) +
            "_" + std::to_string(std::sqrt(minConfigDistance2_)) + ".ikcache");

        ikCache_.clear();
        ikNN_.clear();
        lastSavedCacheSize_ = 0;
        if (boost::filesystem::exists(cacheFileName_))
        {
            // read cache
            boost::filesystem::ifstream cacheFile(cacheFileName_, std::ios_base::binary | std::ios_base::in);
            cacheFile.read((char*)&lastSavedCacheSize_, sizeof(unsigned int));
            unsigned int numDofs;
            cacheFile.read((char*)&numDofs, sizeof(unsigned int));
            unsigned int numTips;
            cacheFile.read((char*)&numTips, sizeof(unsigned int));

            ROS_INFO_NAMED("cached_ik_kinematics_plugin",
                "Found %d IK solutions for a %d-dof system with %d end effectors in %s",
                lastSavedCacheSize_, numDofs, numTips, cacheFileName_.string().c_str());

            unsigned int positionSize = 3 * sizeof(tf2Scalar);
            unsigned int orientationSize = 4 * sizeof(tf2Scalar);
            unsigned int poseSize = positionSize + orientationSize;
            unsigned int configSize = numDofs * sizeof(double);
            unsigned int offsetConf = poseSize * numTips;
            unsigned int bufsize = offsetConf + configSize;
            char *buffer = new char[bufsize * lastSavedCacheSize_];
            IKEntry entry;
            entry.first.resize(numTips);
            entry.second.resize(numDofs);
            cacheFile.read(buffer, bufsize * lastSavedCacheSize_);
            ikCache_.reserve(lastSavedCacheSize_);

            for (unsigned i = 0; i < lastSavedCacheSize_; ++i)
            {
                for (auto &pose : entry.first)
                {
                    memcpy(&pose.position[0], buffer, positionSize);
                    memcpy(&pose.orientation[0], buffer + positionSize, orientationSize);
                    buffer += poseSize;
                }
                memcpy(&entry.second[0], buffer, configSize);
                buffer += configSize;
                ikCache_.push_back(entry);
            }
            std::vector<IKEntry*> ikEntryPtrs(lastSavedCacheSize_);
            for (unsigned int i = 0; i < lastSavedCacheSize_; ++i)
                ikEntryPtrs[i] = &ikCache_[i];
            ikNN_.add(ikEntryPtrs);
            // for debugging purposes:
            //verifyCache();
        }

        numJoints_ = num_joints;

        ROS_INFO_NAMED("cached_ik_kinematics_plugin",
                       "cache file %s initialized!",
                       cacheFileName_.string().c_str());
    }

    double IKCache::configDistance2(
        const std::vector<double> &config1, const std::vector<double> &config2) const
    {
        double dist = 0., diff;
        for (unsigned int i = 0; i < config1.size(); ++i)
        {
            diff = config1[i] - config2[i];
            dist += diff * diff;
        }
        return dist;
    }

    const IKCache::IKEntry &
    IKCache::getBestApproximateIKSolution(const Pose &pose) const
    {
        if (ikCache_.empty())
        {
            static IKEntry dummy = std::make_pair(std::vector<Pose>(1, pose), std::vector<double>(numJoints_, 0.));
            return dummy;
        }
        IKEntry query = std::make_pair(std::vector<Pose>(1, pose), std::vector<double>());
        return *ikNN_.nearest(&query);
    }

    const IKCache::IKEntry &
    IKCache::getBestApproximateIKSolution(const std::vector<Pose> &poses) const
    {
        if (ikCache_.empty())
        {
            static IKEntry dummy = std::make_pair(poses, std::vector<double>(numJoints_, 0.));
            return dummy;
        }
        IKEntry query = std::make_pair(poses, std::vector<double>());
        return *ikNN_.nearest(&query);
    }

    void IKCache::updateCache(
        const IKEntry &nearest, const Pose &pose, const std::vector<double> &config) const
    {
        if (ikCache_.size() < ikCache_.capacity() &&
            (nearest.first[0].distance(pose) > minPoseDistance_ || configDistance2(nearest.second, config) > minConfigDistance2_))
        {
            std::lock_guard<std::mutex> slock(lock_);
            ikCache_.emplace_back(std::vector<Pose>(1u, pose), config);
            ikNN_.add(&ikCache_.back());
            if (ikCache_.size() >= lastSavedCacheSize_ + 500u || ikCache_.size() == maxCacheSize_)
                saveCache();
        }
    }

    void IKCache::updateCache(
        const IKEntry &nearest, const std::vector<Pose> &poses, const std::vector<double> &config) const
    {
        if (ikCache_.size() < ikCache_.capacity())
        {
            bool addToCache = configDistance2(nearest.second, config) > minConfigDistance2_;
            if (!addToCache)
            {
                double dist = 0.;
                for (unsigned int i = 0; i < poses.size(); ++i)
                {
                    dist += nearest.first[i].distance(poses[i]);
                    if (dist > minPoseDistance_)
                    {
                        addToCache = true;
                        break;
                    }
                }
            }
            if (addToCache)
            {
                std::lock_guard<std::mutex> slock(lock_);
                ikCache_.emplace_back(poses, config);
                ikNN_.add(&ikCache_.back());
                if (ikCache_.size() >= lastSavedCacheSize_ + 500u || ikCache_.size() == maxCacheSize_)
                    saveCache();
            }
        }
    }

    void IKCache::saveCache() const
    {
        if (cacheFileName_.empty())
            ROS_ERROR_NAMED("cached_ik_kinematics_plugin",
                "can't save cache before initialization");

        ROS_INFO_NAMED("cached_ik_kinematics_plugin",
            "writing %ld IK solutions to %s",
            ikCache_.size(), cacheFileName_.string().c_str());

        boost::filesystem::ofstream cacheFile(cacheFileName_, std::ios_base::binary | std::ios_base::out);
        unsigned int positionSize = 3 * sizeof(tf2Scalar);
        unsigned int orientationSize = 4 * sizeof(tf2Scalar);
        unsigned int poseSize = positionSize + orientationSize;
        unsigned int numTips = ikCache_[0].first.size();
        unsigned int configSize = ikCache_[0].second.size() * sizeof(double);
        unsigned int offsetConf = numTips * poseSize;
        unsigned int bufsize = offsetConf + configSize;
        char *buffer = new char[bufsize];

        // write number of IK entries and size of each configuration first
        lastSavedCacheSize_ = ikCache_.size();
        cacheFile.write((char*)&lastSavedCacheSize_, sizeof(unsigned int));
        unsigned int sz = ikCache_[0].second.size();
        cacheFile.write((char*)&sz, sizeof(unsigned int));
        cacheFile.write((char*)&numTips, sizeof(unsigned int));
        for (const auto &entry : ikCache_)
        {
            for (unsigned int i = 0; i < numTips; ++i)
            {
                memcpy(buffer + i * poseSize, &entry.first[i].position[0], positionSize);
                memcpy(buffer + i * poseSize + positionSize, &entry.first[i].orientation[0], orientationSize);
            }
            memcpy(buffer + offsetConf, &entry.second[0], configSize);
            cacheFile.write(buffer, bufsize);
        }
    }

    void IKCache::verifyCache(kdl_kinematics_plugin::KDLKinematicsPlugin &fk) const
    {
        std::vector<std::string> tipNames(fk.getTipFrames());
        std::vector<geometry_msgs::Pose> poses(tipNames.size());
        double error, max_error = 0.;

        for (const auto &entry : ikCache_)
        {
            fk.getPositionFK(tipNames, entry.second, poses);
            error = 0.;
            for (unsigned int i = 0; i < poses.size(); ++i)
                error += entry.first[i].distance(poses[i]);
            error /= (double)poses.size();
            if (error > max_error)
                max_error = error;
            if (error > 1e-4)
                ROS_ERROR_NAMED("cached_ik_kinematics_plugin",
                    "Cache entry is invalid, error = %g", error);
        }
        ROS_INFO_NAMED("cached_ik_kinematics_plugin",
            "Max. error in cache entries is %g", max_error);
    }

    IKCache::Pose::Pose(const geometry_msgs::Pose& pose)
    {
        position.setX(pose.position.x);
        position.setY(pose.position.y);
        position.setZ(pose.position.z);
        orientation = tf2::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    }

    double IKCache::Pose::distance(const Pose &pose) const
    {
        return (position - pose.position).length() + (orientation.angleShortestPath(pose.orientation));
    }


    IKCacheMap::IKCacheMap(const std::string &robot_description, const std::string &group_name, unsigned int num_joints)
        : robotDescription_(robot_description), groupName_(group_name), numJoints_(num_joints)
    {
    }

    IKCacheMap::~IKCacheMap()
    {
        for (auto &cache : *this)
            delete cache.second;
    }

    const IKCache::IKEntry &
    IKCacheMap::getBestApproximateIKSolution(const std::vector<std::string> &fixed, const std::vector<std::string> &active,
                                          const std::vector<Pose> &poses) const
    {
        auto key(getKey(fixed, active));
        auto it = find(key);
        if (it != end())
            return it->second->getBestApproximateIKSolution(poses);
        else
        {
            static IKEntry dummy = std::make_pair(poses, std::vector<double>(numJoints_, 0.));
            return dummy;
        }
    }

    void IKCacheMap::updateCache(
        const IKEntry &nearest, const std::vector<std::string> &fixed, const std::vector<std::string> &active,
        const std::vector<Pose> &poses, const std::vector<double> &config)
    {
        auto key(getKey(fixed, active));
        auto it = find(key);
        if (it != end())
            it->second->updateCache(nearest, poses, config);
        else
        {
            value_type val = std::make_pair(key, nullptr);
            auto it = insert(val).first;
            it->second = new IKCache;
            it->second->initializeCache(robotDescription_, groupName_, key, numJoints_);
        }
    }

    std::string IKCacheMap::getKey(const std::vector<std::string> &fixed, const std::vector<std::string> &active) const
    {
        std::string key;
        std::accumulate(fixed.begin(), fixed.end(), key);
        key += '_';
        std::accumulate(active.begin(), active.end(), key);
        return key;
    }

}
