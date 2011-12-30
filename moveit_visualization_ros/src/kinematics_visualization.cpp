#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <planning_scene_monitor/planning_scene_monitor.h>
#include <kinematics_constraint_aware/kinematics_solver_constraint_aware.h>
#include <interactive_markers/interactive_marker_server.h>
#include <tf/tf.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

static const std::string VIS_TOPIC_NAME = "kinematics_visualization";

class KinematicsVisualization {
public:
  
  KinematicsVisualization(boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor>& planning_scene_monitor,
                          boost::shared_ptr<interactive_markers::InteractiveMarkerServer>& interactive_marker_server, 
                          const std::string& group_name, 
                          const std::string& kinematics_solver_name,
                          ros::Publisher& marker_publisher) :
    planning_scene_monitor_(planning_scene_monitor),
    interactive_marker_server_(interactive_marker_server),
    state_(planning_scene_monitor_->getPlanningScene()->getCurrentState()),
    marker_publisher_(marker_publisher)
  {
    const std::map<std::string, srdf::Model::Group>& group_map 
      = planning_scene_monitor_->getPlanningScene()->getKinematicModel()->getJointModelGroupConfigMap();
    
    kinematics_loader_.reset(new pluginlib::ClassLoader<kinematics::KinematicsBase>("kinematics_base", "kinematics::KinematicsBase"));    
    
    if(group_map.find(group_name) == group_map.end()) {
      ROS_ERROR_STREAM("No group named " << group_name);
      return;
    }

    const srdf::Model::Group& srdf_group = group_map.find(group_name)->second;

    if(srdf_group.chains_.size() == 0 ||
       srdf_group.chains_[0].first.empty() ||
       srdf_group.chains_[0].second.empty()) {
      ROS_ERROR_STREAM("Group name " << group_name << " has no or messed up chain definition");
      return;
    }
    
    kinematics::KinematicsBasePtr result;
    result.reset(kinematics_loader_->createClassInstance(kinematics_solver_name));
    
    ik_solver_.reset(new kinematics_constraint_aware::KinematicsSolverConstraintAware(result,
                                                                                      planning_scene_monitor_->getPlanningScene()->getKinematicModel(),
                                                                                      group_name));
    makeInteractiveControlMarker("right_arm_interactive_kinematics",
                                 state_.getLinkState(srdf_group.chains_[0].second)->getGlobalLinkTransform());
    
  };

  void processInteractiveFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) 
  {    
    switch (feedback->event_type) {
    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      updateEndEffectorState(feedback->pose);
      break;
    default:
      ROS_DEBUG_STREAM("Getting event type " << feedback->event_type);
    }
    interactive_marker_server_->applyChanges();
  }; 
  
  void makeInteractiveControlMarker(const std::string& name,
                                    const btTransform& transform) 
  {
    visualization_msgs::InteractiveMarker marker;
    marker.header.frame_id = "/" + planning_scene_monitor_->getPlanningScene()->getPlanningFrame();
    marker.pose.position.x = transform.getOrigin().x();
    marker.pose.position.y = transform.getOrigin().y();
    marker.pose.position.z = transform.getOrigin().z();
    marker.pose.orientation.w = transform.getRotation().w();
    marker.pose.orientation.x = transform.getRotation().x();
    marker.pose.orientation.y = transform.getRotation().y();
    marker.pose.orientation.z = transform.getRotation().z();
    marker.scale = .225;
    marker.name = name;
    marker.description = "ik";

    visualization_msgs::InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    visualization_msgs::Marker box_marker;
    box_marker.type = visualization_msgs::Marker::CUBE;
    // Scale is arbitrarily 1/4 of the marker's scale.
    box_marker.scale.x = .1 * 0.25;
    box_marker.scale.y = .1 * 0.25;
    box_marker.scale.z = .1 * 0.25;
    box_marker.color.r = 1.0;
    box_marker.color.g = 1.0;
    box_marker.color.b = 1.0;
    box_marker.color.a = 1.0;

    box_control.markers.push_back(box_marker);
    marker.controls.push_back(box_control);
    
    visualization_msgs::InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.always_visible = false;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    marker.controls.push_back(control);
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    marker.controls.push_back(control);
    
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    marker.controls.push_back(control);
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    marker.controls.push_back(control);
    
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    marker.controls.push_back(control);
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    marker.controls.push_back(control);
    
    interactive_marker_server_->insert(marker);
    interactive_marker_server_->setCallback(marker.name, 
                                            boost::bind(&KinematicsVisualization::processInteractiveFeedback, this, _1));
    interactive_marker_server_->applyChanges();
  };
      
  void updateEndEffectorState(const geometry_msgs::Pose& pose) {
    
    //assuming pose is in world frame for now
    btTransform cur;
    tf::poseMsgToTF(pose, cur);

    state_.updateStateWithLinkAt(ik_solver_->getTipFrame(), cur);
    
    //now need to get in base_frame
    btTransform base_in_world = state_.getLinkState(ik_solver_->getBaseFrame())->getGlobalLinkTransform();
    
    ROS_DEBUG_STREAM("Base x y z " << base_in_world.getOrigin().x() << " " 
                    << base_in_world.getOrigin().y() << " " 
                    << base_in_world.getOrigin().z()); 

    btTransform tip_in_base = base_in_world.inverse()*cur;
    
    geometry_msgs::Pose np;
    tf::poseTFToMsg(tip_in_base, np);

    ROS_DEBUG_STREAM("X Y Z are " << np.position.x << " " 
                     << np.position.y << " " 
                     << np.position.z << " from " << ik_solver_->getBaseFrame()); 

    sensor_msgs::JointState sol;
    moveit_msgs::MoveItErrorCodes err;

    std_msgs::ColorRGBA good_color;
    good_color.a = 1.0;    
    good_color.g = 1.0;    

    std_msgs::ColorRGBA bad_color;
    bad_color.a = 1.0;    
    bad_color.g = 1.0;    

    visualization_msgs::MarkerArray arr;
    if(ik_solver_->getPositionIK(np,
                                 planning_scene_monitor_->getPlanningScene(),
                                 sol,
                                 err)) {
      state_.setStateValues(sol);
      visualization_msgs::MarkerArray mark;
      state_.getRobotMarkers(good_color,
                             "ik_solution",
                             ros::Duration(0.0),
                             mark);
      marker_publisher_.publish(mark);
    } else {
      ROS_INFO_STREAM("IK not ok " << err.val);
    }
  }

  ~KinematicsVisualization() {
  }
  
protected:

  boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_marker_server_;
  planning_models::KinematicState state_;
  ros::Publisher marker_publisher_;
  
  boost::shared_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase> > kinematics_loader_;
  boost::shared_ptr<kinematics_constraint_aware::KinematicsSolverConstraintAware> ik_solver_;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinematics_visualization", ros::init_options::NoSigintHandler);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_marker_server;
  interactive_marker_server.reset(new interactive_markers::InteractiveMarkerServer("interactive_kinematics_visualization", "", false));

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

  KinematicsVisualization::KinematicsVisualization kv(planning_scene_monitor,
                                                      interactive_marker_server,
                                                      "right_arm",
                                                      "pr2_arm_kinematics/PR2ArmKinematicsPlugin",
                                                      vis_marker_array_publisher);

  // geometry_msgs::PoseStamped pose;
  // pose.pose.position.x = .58;
  // pose.pose.position.y = -.18;
  // pose.pose.position.z = .75;
  // pose.pose.orientation.x = 0.0;
  // pose.pose.orientation.y = 0.88793;
  // pose.pose.orientation.z = 0.0;
  // pose.pose.orientation.w = -.459979;

  // kv.updateEndEffectorState("right_arm",
  //                           pose);

  ros::waitForShutdown();
}

