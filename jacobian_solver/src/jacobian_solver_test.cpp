

#include <ros/ros.h>
#include <jacobian_solver/jacobian_solver.h>
#include <jacobian_solver/kinematic_group_jacobian_solver.h>
//#include <planning_models/conversions.h>

class JacobianTest {

  //typedef jacobian_solver::KinematicGroupJacobianSolver JacobianSolver;

public:
  void setup()
  {
    urdf_model_.reset(new urdf::Model());
    srdf_model_.reset(new srdf::Model());
    urdf_ok_ = urdf_model_->initFile("test/urdf/robot.xml");
    srdf_ok_ = srdf_model_->initFile(*urdf_model_, "test/srdf/robot.xml");

    kmodel_.reset(new planning_models::KinematicModel(urdf_model_, srdf_model_));

    jac_solver_.reset(new jacobian_solver::JacobianSolver());
    jac_solver_->initialize(urdf_model_, srdf_model_, "right_arm");

    group_jac_solver_.reset(new jacobian_solver::KinematicGroupJacobianSolver());
    group_jac_solver_->initialize(urdf_model_, srdf_model_, "right_arm");

  }

  void run(uint64_t num_tests){
    for(int method = 0; method < 2; method++)
    {
      ros::Time begin = ros::Time::now();
      uint64_t num = num_tests;
      for(uint64_t i = 0; i < num; i++)
      {
        bool print = (i == 0);
        compute("r_shoulder_pan_link", print, method);
        compute("r_shoulder_lift_link", print, method);
        compute("r_upper_arm_roll_link", print, method);
        compute("r_elbow_flex_link", print, method);
        compute("r_forearm_roll_link", print, method);
        compute("r_wrist_flex_link", print, method);
        compute("r_wrist_roll_link", print, method);
      }
      ros::Time end = ros::Time::now();
      ROS_INFO_COND( method == 0, "Recursive jacobian method:");
      ROS_INFO_COND( method == 1, "Map of chains method:");
      ROS_INFO_STREAM("Each computation cycle took an average of " << (end - begin).toNSec()/num/1000.0 << " us.");
      ROS_INFO_STREAM(num << " iterations took " << (end - begin).toNSec()/1000 << " us.");
    }
  }

  void compute(const std::string &link, bool print, int method){

    double angles[] = {0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25};
    std::vector<double> joint_angles(angles, angles + sizeof(angles) / sizeof(double));

    Eigen::Vector3d point(0.2,0,0);
    Eigen::MatrixXd jacobian;

    if(method == 0)
      jac_solver_->getJacobian(link, joint_angles, point, jacobian);
    if(method == 1)
      group_jac_solver_->getJacobian(link, joint_angles, point, jacobian);

    ROS_INFO_STREAM_COND(print, "\npoint: " << point(0)<<" "<<point(1)<<" "<<point(2)<< " on link " << link << " has jacobian: \n" << jacobian);
  }

protected:
  bool urdf_ok_;
  bool srdf_ok_;

  boost::shared_ptr<urdf::Model>           urdf_model_;
  boost::shared_ptr<srdf::Model>           srdf_model_;

  planning_models::KinematicModelPtr             kmodel_;

  boost::shared_ptr<jacobian_solver::KinematicGroupJacobianSolver> group_jac_solver_;
  boost::shared_ptr<jacobian_solver::JacobianSolver> jac_solver_;

};







int main(int argc, char **argv){

  ros::init(argc, argv, "jacobian_solver_test");
  ros::start();

  JacobianTest test;
  test.setup();

  test.run(atoi(argv[1]));

  //ros::spin();

  return 0;
}
