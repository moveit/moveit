
#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <planning_scene_monitor/planning_scene_monitor.h>
#include <kinematics_constraint_aware/kinematics_solver_constraint_aware.h>
#include <tf/tf.h>

static const std::string VIS_TOPIC_NAME = "kinematics_visualization";

class KinematicsVisualization
{
public:

  KinematicsVisualization(boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor>& planning_scene_monitor,
                          const std::string& kinematics_solver_name,
                          ros::Publisher& marker_publisher) :
    marker_publisher_(marker_publisher),
    planning_scene_monitor_(planning_scene_monitor),
    state_(planning_scene_monitor_->getPlanningScene()->getCurrentState())
  {
    const std::map<std::string, srdf::Model::Group>& group_map 
      = planning_scene_monitor_->getPlanningScene()->getKinematicModel()->getJointModelGroupConfigMap();
    
    kinematics_loader_.reset(new pluginlib::ClassLoader<kinematics::KinematicsBase>("kinematics_base", "kinematics::KinematicsBase"));    

    for(std::map<std::string, srdf::Model::Group>::const_iterator it = group_map.begin();
        it != group_map.end(); 
        it++) {

      if(it->second.chains_.size() == 0) continue;
      if(it->second.chains_[0].first.empty() ||
         it->second.chains_[0].second.empty()) continue;

      kinematics::KinematicsBasePtr result;
      result.reset(kinematics_loader_->createClassInstance(kinematics_solver_name));

      ik_solvers_[it->first].reset(new kinematics_constraint_aware::KinematicsSolverConstraintAware(result,
                                                                                                    planning_scene_monitor_->getPlanningScene()->getKinematicModel(),
                                                                                                    it->first));
    }
  }

  void updateEndEffectorState(const std::string& group_name,
                              const geometry_msgs::PoseStamped& pose) {
    
    //assuming pose is in world frame for now
    btTransform cur;
    tf::poseMsgToTF(pose.pose, cur);

    state_.updateStateWithLinkAt(ik_solvers_[group_name]->getTipFrame(), cur);
    
    //now need to get in base_frame
    btTransform base_in_world = state_.getLinkState(ik_solvers_[group_name]->getBaseFrame())->getGlobalLinkTransform();
    
    ROS_INFO_STREAM("Base x y z " << base_in_world.getOrigin().x() << " " 
                    << base_in_world.getOrigin().y() << " " 
                    << base_in_world.getOrigin().z()); 

    btTransform tip_in_base = base_in_world.inverse()*cur;
    
    geometry_msgs::Pose np;
    tf::poseTFToMsg(tip_in_base, np);

    ROS_INFO_STREAM("X Y Z are " << np.position.x << " " 
                    << np.position.y << " " 
                    << np.position.z << " from " << ik_solvers_[group_name]->getBaseFrame()); 

    sensor_msgs::JointState sol;
    moveit_msgs::MoveItErrorCodes err;

    std_msgs::ColorRGBA good_color;
    good_color.a = 1.0;    
    good_color.g = 1.0;    

    std_msgs::ColorRGBA bad_color;
    bad_color.a = 1.0;    
    bad_color.g = 1.0;    

    visualization_msgs::MarkerArray arr;
    if(ik_solvers_[group_name]->getPositionIK(np,
                                              planning_scene_monitor_->getPlanningScene(),
                                              sol,
                                              err)) {
      state_.setStateValues(sol);
      visualization_msgs::MarkerArray mark;
      state_.getRobotMarkers(good_color,
                             "ik_solution",
                             ros::Duration(0.0),
                             mark);
      while(ros::ok()) {
        marker_publisher_.publish(mark);
        ros::WallDuration(.1).sleep();
      }
    } else {
      ROS_INFO_STREAM("IK not ok " << err.val);
    }
  }


  ~KinematicsVisualization() {
    ik_solvers_.clear();
  }
  
protected:

  ros::Publisher marker_publisher_;

  boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;

  planning_models::KinematicState state_;

  boost::shared_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase> > kinematics_loader_;
  std::map<std::string, boost::shared_ptr<kinematics_constraint_aware::KinematicsSolverConstraintAware> > ik_solvers_;


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinematics_visualization", ros::init_options::NoSigintHandler);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  //boost::thread spin_thread(boost::bind(&spin_function));
  //boost::thread update_thread(boost::bind(&update_function));

  tf::TransformListener listener;
  
  boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor;
  planning_scene_monitor.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description", &listener));

  planning_scene_monitor->startWorldGeometryMonitor();
  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startStateMonitor();

  while(ros::ok()) {
    if(planning_scene_monitor->getStateMonitor()->haveCompleteState()) {
      ROS_INFO_STREAM("Have full state");
      break;
    }
    ROS_INFO_STREAM("Waiting for state");
    ros::WallDuration(.1).sleep();
  }

  planning_scene_monitor->updateSceneWithCurrentState();

  ros::NodeHandle nh;

  ros::Publisher vis_marker_array_publisher;
  ros::Publisher vis_marker_publisher;

  vis_marker_publisher = nh.advertise<visualization_msgs::Marker> (VIS_TOPIC_NAME, 128);
  vis_marker_array_publisher = nh.advertise<visualization_msgs::MarkerArray> (VIS_TOPIC_NAME + "_array", 128);

  KinematicsVisualization kv(planning_scene_monitor,
                             "pr2_arm_kinematics/PR2ArmKinematicsPlugin",
                             vis_marker_array_publisher);


  
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = .58;
  pose.pose.position.y = -.18;
  pose.pose.position.z = .75;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.88793;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = -.459979;
  
  kv.updateEndEffectorState("right_arm",
                            pose);

  ros::waitForShutdown();
}
