/*
 * chomp_planning_context.h
 *
 *  Created on: 27-Jul-2016
 *      Author: ace
 */

#ifndef CHOMP_PLANNING_CONTEXT_H_
#define CHOMP_PLANNING_CONTEXT_H_

#include <moveit/planning_interface/planning_interface.h>
#include <chomp_interface/chomp_interface.h>
#include <chomp_interface/chomp_planning_context.h>

#include <moveit/robot_state/conversions.h>

namespace chomp_interface {

class ChompPlanningContext: public planning_interface::PlanningContext {

  typedef boost::shared_ptr <ChompPlanningContext> Ptr;
public:

  virtual bool solve(planning_interface::MotionPlanResponse &res);
  virtual bool solve(planning_interface::MotionPlanDetailedResponse &res);

  virtual void clear();
  virtual bool terminate();

  ChompPlanningContext(const std::string &name, const std::string &group, const robot_model::RobotModelConstPtr& model) :
    planning_interface::PlanningContext(name, group),
    kmodel_ (model) {
    chomp_interface_ = boost::shared_ptr <CHOMPInterface> (new CHOMPInterface(model));
  }

  virtual ~ChompPlanningContext();

  void initialize();

private:
  boost::shared_ptr<CHOMPInterface> chomp_interface_;
  moveit::core::RobotModelConstPtr kmodel_;
};

typedef boost::shared_ptr<ChompPlanningContext> ChompPlanningContextPtr;

} /* namespace chomp_interface */

#endif /* CHOMP_PLANNING_CONTEXT_H_ */
