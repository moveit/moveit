#pragma once
#include <tesseract_core/basic_env.h>
#include <tesseract_core/basic_kin.h>
#include <trajopt/common.hpp>
#include <trajopt/json_marshal.hpp>
#include <trajopt_sco/optimizers.hpp>
#include <moveit/robot_model/robot_model.h>

namespace trajopt_interface
{

class TrajOptProblem;
typedef std::shared_ptr<TrajOptProblem> TrajOptProblemPtr;

/**
 * Holds all the data for a trajectory optimization problem
 * so you can modify it programmatically, e.g. add your own costs
 */
class  TrajOptProblem :  public sco::OptProb
{
public:
  TrajOptProblem();
  TrajOptProblem(const int& n_steps, bool& use_time, const robot_model::RobotModelConstPtr& robot_model);
  virtual ~TrajOptProblem() = default;
  sco::VarVector GetVarRow(int i, int start_col, int num_col) { return m_traj_vars.rblock(i, start_col, num_col); }
  sco::VarVector GetVarRow(int i) { return m_traj_vars.row(i); }
  sco::Var& GetVar(int i, int j) { return m_traj_vars.at(i, j); }
  trajopt::VarArray& GetVars() { return m_traj_vars; }
  /** @brief Returns the number of steps in the problem. This is the number of rows in the optimization matrix.*/
  int GetNumSteps() { return m_traj_vars.rows(); }
  /** @brief Returns the problem DOF. This is the number of columns in the optization matrix.
   * Note that this is not necessarily the same as the kinematic DOF.*/
  int GetNumDOF() { return m_traj_vars.cols(); }
  // tesseract::BasicKinConstPtr GetKin() { return m_kin; }
  //tesseract::BasicEnvConstPtr GetEnv() { return m_env; }
  void SetInitTraj(const trajopt::TrajArray& x){
    std::cout << "====>>>>> setInitTraj is called " << std::endl;
    std::cout << "x.size: " << x.size()  << std::endl;
    m_init_traj = x;
    //    std::cout << m_init_traj << std::endl;
  }
  trajopt::TrajArray GetInitTraj() {
    std::cout << "====>>>>> GetInitTraj is called " << std::endl;
    return m_init_traj;
  }
  //  friend TrajOptProbPtr ConstructProblem(const ProblemConstructionInfo&);
  /** @brief Returns TrajOptProb.has_time */
  bool GetHasTime() { return has_time; }
  /** @brief Sets TrajOptProb.has_time  */
  void SetHasTime(bool tmp) { has_time = tmp; }

private:
  /** @brief If true, the last column in the optimization matrix will be 1/dt */
  bool has_time;
  trajopt::VarArray m_traj_vars;
  // tesseract::BasicKinConstPtr m_kin;
  // tesseract::BasicEnvConstPtr m_env;
  trajopt::TrajArray m_init_traj;
};

/**
When cost or constraint element of JSON doc is read, one of these guys gets
constructed to hold the parameters.
Then it later gets converted to a Cost object by the hatch method
*/
struct TermInfo
{
  std::string name;
  int term_type;
  int getSupportedTypes() { return supported_term_types_; }
  //  virtual void fromJson(ProblemConstructionInfo& pci, const Json::Value& v) = 0;
  virtual void hatch(TrajOptProblem& prob) = 0;

  //  static TermInfoPtr fromName(const std::string& type);

  /**
   * Registers a user-defined TermInfo so you can use your own cost
   * see function RegisterMakers.cpp
   */
  //  typedef TermInfoPtr (*MakerFunc)(void);
  //  static void RegisterMaker(const std::string& type, MakerFunc);

  virtual ~TermInfo() = default;

protected:
  TermInfo(int supported_term_types) : supported_term_types_(supported_term_types) {}
private:
  //  static std::map<std::string, MakerFunc> name2maker;
  int supported_term_types_;
};

struct CartPoseTermInfo : public TermInfo
{
  //  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** @brief Timestep at which to apply term */
  int timestep;
  /** @brief  Cartesian position */
  Eigen::Vector3d xyz;
  /** @brief Rotation quaternion */
  Eigen::Vector4d wxyz;
  /** @brief coefficients for position and rotation */
  Eigen::Vector3d pos_coeffs, rot_coeffs;
  /** @brief Link which should reach desired pose */
  std::string link;
  /** @brief Static transform applied to the link */
  Eigen::Isometry3d tcp;

  CartPoseTermInfo();

  /** @brief Used to add term to pci from json */
  //  void fromJson(ProblemConstructionInfo& pci, const Json::Value& v) override;
  /** @brief Converts term info into cost/constraint and adds it to trajopt problem */
  void hatch(TrajOptProblem& prob) override;
  // DEFINE_CREATE(CartPoseTermInfo)
};



}  // namespace trajopt_interface
