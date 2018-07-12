/**
 * @file update_logger.cpp
 * @brief This defines a update trajectory logger which writes the data to a file.
 *
 * @author Jorge Nicho
 * @date April 13, 2016
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2016, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stomp_moveit/update_filters/update_logger.h>
#include <boost/filesystem.hpp>
#include <ros/console.h>
#include <pluginlib/class_list_macros.h>
#include <ros/package.h>
#include <Eigen/Core>

PLUGINLIB_EXPORT_CLASS(stomp_moveit::update_filters::UpdateLogger, stomp_moveit::update_filters::StompUpdateFilter);

namespace stomp_moveit
{
namespace update_filters
{
UpdateLogger::UpdateLogger() : name_("UpdateLogger")
{
}

UpdateLogger::~UpdateLogger()
{
  // TODO Auto-generated destructor stub
}

bool UpdateLogger::initialize(moveit::core::RobotModelConstPtr robot_model_ptr, const std::string& group_name,
                              const XmlRpc::XmlRpcValue& config)
{
  format_ = Eigen::IOFormat(Eigen::StreamPrecision, 0, " ", "\n");
  group_name_ = group_name;
  return configure(config);
}

bool UpdateLogger::configure(const XmlRpc::XmlRpcValue& config)
{
  // check parameter presence
  auto members = { "filename", "directory", "package" };
  for (auto& m : members)
  {
    if (!config.hasMember(m))
    {
      ROS_ERROR("%s failed to find one or more required parameters", getName().c_str());
      return false;
    }
  }

  XmlRpc::XmlRpcValue c = config;
  try
  {
    filename_ = static_cast<std::string>(c["filename"]);
    directory_ = static_cast<std::string>(c["directory"]);
    package_ = static_cast<std::string>(c["package"]);
  }
  catch (XmlRpc::XmlRpcException& e)
  {
    ROS_ERROR("%s failed to find the required parameters", getName().c_str());
    return false;
  }

  return true;
}

bool UpdateLogger::setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                        const moveit_msgs::MotionPlanRequest& req,
                                        const stomp_core::StompConfiguration& config,
                                        moveit_msgs::MoveItErrorCodes& error_code)
{
  using namespace boost::filesystem;

  stomp_config_ = config;

  std::string full_dir_name = ros::package::getPath(package_) + "/" + directory_;
  full_file_name_ = full_dir_name + "/" + filename_;
  path dir_path(full_dir_name);

  if (!boost::filesystem::is_directory(dir_path))
  {
    // create directory
    if (!boost::filesystem::create_directory(dir_path))
    {
      ROS_ERROR("Unable to create the update logging directory in the path %s", full_dir_name.c_str());
      return false;
    }
  }

  // open file
  file_stream_.open(full_file_name_);
  if (!file_stream_.is_open())
  {
    ROS_ERROR("Unable to create/open update log file %s", full_file_name_.c_str());
    return false;
  }

  // clear stream
  stream_.str("");

  return true;
}

bool UpdateLogger::filter(std::size_t start_timestep, std::size_t num_timesteps, int iteration_number,
                          const Eigen::MatrixXd& parameters, Eigen::MatrixXd& updates, bool& filtered)
{
  stream_ << updates.format(format_) << std::endl;
  filtered = false;
  return true;
}

void UpdateLogger::done(bool success, int total_iterations, double final_cost, const Eigen::MatrixXd& parameters)
{
  // creating header
  std::string header = R"(# num_iterations: @iterations
# num_timesteps: @timesteps
# num_dimensions: @dimensions
# matrix_rows: @rows
# matrix_cols: @cols)";

  // replacing values into header
  int rows = stomp_config_.num_dimensions * total_iterations;
  int cols = stomp_config_.num_timesteps;
  header.replace(header.find("@iterations"), std::string("@iterations").length(), std::to_string(total_iterations));
  header.replace(header.find("@timesteps"), std::string("@timesteps").length(),
                 std::to_string(stomp_config_.num_timesteps));
  header.replace(header.find("@dimensions"), std::string("@dimensions").length(),
                 std::to_string(stomp_config_.num_dimensions));
  header.replace(header.find("@rows"), std::string("@rows").length(), std::to_string(rows));
  header.replace(header.find("@cols"), std::string("@cols").length(), std::to_string(cols));

  // writing to file stream
  file_stream_ << header << std::endl;
  file_stream_ << stream_.str();
  file_stream_.close();

  // clear
  stream_.str("");

  ROS_INFO("Saved update log file %s, read with 'numpy.loadtxt(\"%s\")'", full_file_name_.c_str(), filename_.c_str());
}

} /* namespace smoothers */
} /* namespace stomp_moveit */
