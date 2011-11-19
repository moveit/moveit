
#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <planning_scene_ros/planning_scene_ros.h>
#include <kinematics_constraint_aware/kinematics_solver_constraint_aware.h>

class KinematicsVisualization
{
public:

  KinematicsVisualization(boost::shared_ptr<planning_scene_ros::PlanningSceneROS>& planning_scene,
                          const std::string& kinematics_solver_name) :
    planning_scene_(planning_scene),
    state_(planning_scene->getCurrentState())
  {
    const std::map<std::string, srdf::Model::Group>& group_map 
      = planning_scene->getKinematicModel()->getJointModelGroupConfigMap();
    
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
                                                                                                    planning_scene->getKinematicModel(),
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
    btTransform tip_in_base = cur.inverse()*base_in_world;
    
    geometry_msgs::Pose np;
    tf::poseTFToMsg(tip_in_base, np);

    ROS_INFO_STREAM("X Y Z are " << np.position.x << " " 
                    << np.position.y << " " 
                    << np.position.z << " from " << ik_solvers_[group_name]->getBaseFrame()); 

    sensor_msgs::JointState sol;
    moveit_msgs::MoveItErrorCodes err;
    if(ik_solvers_[group_name]->getPositionIK(np,
                                              planning_scene_,
                                              sol,
                                              err)) {
      ROS_INFO_STREAM("IK ok");
    } else {
      ROS_INFO_STREAM("IK not ok " << err.val);
    }
  }


  ~KinematicsVisualization() {
    ik_solvers_.clear();
  }
  
protected:

  boost::shared_ptr<planning_scene_ros::PlanningSceneROS> planning_scene_;

  planning_models::KinematicState state_;

  boost::shared_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase> > kinematics_loader_;
  std::map<std::string, boost::shared_ptr<kinematics_constraint_aware::KinematicsSolverConstraintAware> > ik_solvers_;


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinematics_visualization", ros::init_options::NoSigintHandler);

  //boost::thread spin_thread(boost::bind(&spin_function));
  //boost::thread update_thread(boost::bind(&update_function));
  
  boost::shared_ptr<planning_scene_ros::PlanningSceneROS> planning_scene_ros;
  planning_scene_ros.reset(new planning_scene_ros::PlanningSceneROS("robot_description"));

  KinematicsVisualization kv(planning_scene_ros,
                             "pr2_arm_kinematics/PR2ArmKinematicsPlugin");

  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = .58;
  pose.pose.position.y = -.18;
  pose.pose.position.z = 1.25;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.88793;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = -.459979;

  kv.updateEndEffectorState("right_arm",
                            pose);
}
