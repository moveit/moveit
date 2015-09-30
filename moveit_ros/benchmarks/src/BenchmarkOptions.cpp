#include "moveit/benchmarks/BenchmarkOptions.h"

using namespace moveit_benchmarks;

BenchmarkOptions::BenchmarkOptions()
{
}

BenchmarkOptions::BenchmarkOptions(const std::string& ros_namespace)
{
    readBenchmarkOptions(ros_namespace);
}

BenchmarkOptions::~BenchmarkOptions()
{
}

void BenchmarkOptions::setNamespace(const std::string& ros_namespace)
{
    readBenchmarkOptions(ros_namespace);
}

void BenchmarkOptions::readBenchmarkOptions(const std::string& ros_namespace)
{
    ros::NodeHandle nh(ros_namespace);

    XmlRpc::XmlRpcValue benchmark_config;
    if (nh.getParam("benchmark_config", benchmark_config))
    {
        readWarehouseOptions(nh);
        readBenchmarkParameters(nh);
        readPlannerConfigs(nh);
    }
    else
    {
        ROS_WARN("No benchmark_config found on param server");
    }
}

const std::string& BenchmarkOptions::getHostName() const
{
    return hostname_;
}

int BenchmarkOptions::getPort() const
{
    return port_;
}

const std::string& BenchmarkOptions::getSceneName() const
{
    return scene_name_;
}

int BenchmarkOptions::getNumRuns() const
{
    return runs_;
}

double BenchmarkOptions::getTimeout() const
{
    return timeout_;
}

const std::string& BenchmarkOptions::getBenchmarkName() const
{
    return benchmark_name_;
}

const std::string& BenchmarkOptions::getGroupName() const
{
    return group_name_;
}

const std::string& BenchmarkOptions::getOutputDirectory() const
{
    return output_directory_;
}

const std::string& BenchmarkOptions::getQueryRegex() const
{
    return query_regex_;
}

const std::string& BenchmarkOptions::getStartStateRegex() const
{
    return start_state_regex_;
}

const std::string& BenchmarkOptions::getGoalConstraintRegex() const
{
    return goal_constraint_regex_;
}

const std::string& BenchmarkOptions::getPathConstraintRegex() const
{
    return path_constraint_regex_;
}

const std::string& BenchmarkOptions::getTrajectoryConstraintRegex() const
{
    return trajectory_constraint_regex_;
}

void BenchmarkOptions::getGoalOffsets(std::vector<double>& offsets) const
{
    offsets.resize(6);
    memcpy(&offsets[0], goal_offsets, 6 * sizeof(double));
}

const std::map<std::string, std::vector<std::string> >& BenchmarkOptions::getPlannerConfigurations() const
{
    return planners_;
}

void BenchmarkOptions::getPlannerPluginList(std::vector<std::string>& plugin_list) const
{
    plugin_list.clear();
    for(std::map<std::string, std::vector<std::string> >::const_iterator it = planners_.begin(); it != planners_.end(); ++it)
        plugin_list.push_back(it->first);
}

const std::string& BenchmarkOptions::getWorkspaceFrameID() const
{
    return workspace_.header.frame_id;
}

const moveit_msgs::WorkspaceParameters& BenchmarkOptions::getWorkspaceParameters() const
{
    return workspace_;
}

void BenchmarkOptions::readWarehouseOptions(ros::NodeHandle& nh)
{
    nh.param(std::string("benchmark_config/warehouse/host"), hostname_, std::string("127.0.0.1"));
    nh.param(std::string("benchmark_config/warehouse/port"), port_, 33829);

    if (!nh.getParam("benchmark_config/warehouse/scene_name", scene_name_))
        ROS_WARN("Benchmark scene_name NOT specified");


    ROS_INFO("Benchmark host: %s", hostname_.c_str());
    ROS_INFO("Benchmark port: %d", port_);
    ROS_INFO("Benchmark scene: %s", scene_name_.c_str());
}

void BenchmarkOptions::readBenchmarkParameters(ros::NodeHandle& nh)
{
    nh.param(std::string("benchmark_config/parameters/name"), benchmark_name_, std::string(""));
    nh.param(std::string("benchmark_config/parameters/runs"), runs_, 10);
    nh.param(std::string("benchmark_config/parameters/timeout"), timeout_, 10.0);
    nh.param(std::string("benchmark_config/parameters/output_directory"), output_directory_, std::string(""));
    nh.param(std::string("benchmark_config/parameters/queries"), query_regex_, std::string(".*"));
    nh.param(std::string("benchmark_config/parameters/start_states"), start_state_regex_, std::string(""));
    nh.param(std::string("benchmark_config/parameters/goal_constraints"), goal_constraint_regex_, std::string(""));
    nh.param(std::string("benchmark_config/parameters/path_constraints"), path_constraint_regex_, std::string(""));
    nh.param(std::string("benchmark_config/parameters/trajectory_constraints"), trajectory_constraint_regex_, std::string(""));

    if (!nh.getParam(std::string("benchmark_config/parameters/group"), group_name_))
        ROS_WARN("Benchmark group NOT specified");

    if (nh.hasParam("benchmark_config/parameters/workspace"))
        readWorkspaceParameters(nh);

    // Reading in goal_offset (or defaulting to zero)
    nh.param(std::string("benchmark_config/parameters/goal_offset/x"), goal_offsets[0], 0.0);
    nh.param(std::string("benchmark_config/parameters/goal_offset/y"), goal_offsets[1], 0.0);
    nh.param(std::string("benchmark_config/parameters/goal_offset/z"), goal_offsets[2], 0.0);
    nh.param(std::string("benchmark_config/parameters/goal_offset/roll"),   goal_offsets[3], 0.0);
    nh.param(std::string("benchmark_config/parameters/goal_offset/pitch"),  goal_offsets[4], 0.0);
    nh.param(std::string("benchmark_config/parameters/goal_offset/yaw"),    goal_offsets[5], 0.0);

    ROS_INFO("Benchmark name: '%s'", benchmark_name_.c_str());
    ROS_INFO("Benchmark #runs: %d", runs_);
    ROS_INFO("Benchmark timeout: %f secs", timeout_);
    ROS_INFO("Benchmark group: %s", group_name_.c_str());
    ROS_INFO("Benchmark query regex: '%s'", query_regex_.c_str());
    ROS_INFO("Benchmark start state regex: '%s':", start_state_regex_.c_str());
    ROS_INFO("Benchmark goal constraint regex: '%s':", goal_constraint_regex_.c_str());
    ROS_INFO("Benchmark path constraint regex: '%s':", path_constraint_regex_.c_str());
    ROS_INFO("Benchmark goal offsets (%f %f %f, %f %f %f)", goal_offsets[0], goal_offsets[1], goal_offsets[2], goal_offsets[3], goal_offsets[4], goal_offsets[5]);
    ROS_INFO("Benchmark output directory: %s", output_directory_.c_str());
    ROS_INFO_STREAM("Benchmark workspace: " << workspace_);
}

void BenchmarkOptions::readWorkspaceParameters(ros::NodeHandle& nh)
{
    // Make sure all params exist
    if (!nh.getParam("benchmark_config/parameters/workspace/frame_id", workspace_.header.frame_id))
        ROS_WARN("Workspace frame_id not specified in benchmark config");

    nh.param(std::string("benchmark_config/parameters/workspace/min_corner/x"), workspace_.min_corner.x, 0.0);
    nh.param(std::string("benchmark_config/parameters/workspace/min_corner/y"), workspace_.min_corner.y, 0.0);
    nh.param(std::string("benchmark_config/parameters/workspace/min_corner/z"), workspace_.min_corner.z, 0.0);

    nh.param(std::string("benchmark_config/parameters/workspace/max_corner/x"), workspace_.max_corner.x, 0.0);
    nh.param(std::string("benchmark_config/parameters/workspace/max_corner/y"), workspace_.max_corner.y, 0.0);
    nh.param(std::string("benchmark_config/parameters/workspace/max_corner/z"), workspace_.max_corner.z, 0.0);

    workspace_.header.stamp = ros::Time::now();
}

void BenchmarkOptions::readPlannerConfigs(ros::NodeHandle& nh)
{
    planners_.clear();

    XmlRpc::XmlRpcValue planner_configs;
    if (nh.getParam("benchmark_config/planners", planner_configs))
    {
        if (planner_configs.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_ERROR("Expected a list of planner configurations to benchmark");
            return;
        }

        for(size_t i = 0; i < planner_configs.size(); ++i)
        {
            if (planner_configs[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
            {
                ROS_WARN("Improper YAML type for planner configurations");
                continue;
            }
            if (!planner_configs[i].hasMember("plugin") || !planner_configs[i].hasMember("planners"))
            {
                ROS_WARN("Malformed YAML for planner configurations");
                continue;
            }

            if (planner_configs[i]["planners"].getType() != XmlRpc::XmlRpcValue::TypeArray)
            {
                ROS_WARN("Expected a list of planners to benchmark");
                continue;
            }

            std::string plugin = planner_configs[i]["plugin"];
            ROS_INFO("Reading in planner names for plugin '%s'", plugin.c_str());

            std::vector<std::string> planners;
            for(size_t j = 0; j < planner_configs[i]["planners"].size(); ++j)
                planners.push_back(planner_configs[i]["planners"][j]);

            for(size_t j = 0; j < planners.size(); ++j)
                ROS_INFO("  [%lu]: %s", j, planners[j].c_str());

            planners_[plugin] = planners;
        }
    }
}