/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Ioan Sucan */

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/py_bindings_tools/roscpp_initializer.h>
#include <moveit/py_bindings_tools/py_conversions.h>

#include <boost/function.hpp>
#include <boost/python.hpp>
#include <Python.h>

/** @cond IGNORE */

namespace bp = boost::python;

namespace moveit
{
namespace planning_interface
{

class PlanningSceneInterfaceWrapper : protected py_bindings_tools::ROScppInitializer,
                                      public PlanningSceneInterface
{
public:

  // ROSInitializer is constructed first, and ensures ros::init() was called, if needed
  PlanningSceneInterfaceWrapper() : py_bindings_tools::ROScppInitializer(),
                                    PlanningSceneInterface()
  {
  }

  bp::list getKnownObjectNamesPython(bool with_type = false)
  {
    return moveit::py_bindings_tools::listFromString(getKnownObjectNames(with_type));
  }

  bp::list getKnownObjectNamesInROIPython(double minx, double miny, double minz, double maxx, double maxy, double maxz, bool with_type = false)
  {
    return moveit::py_bindings_tools::listFromString(getKnownObjectNamesInROI(minx, miny, minz, maxx, maxy, maxz, with_type));
  }

};

static void wrap_planning_scene_interface()
{
  bp::class_<PlanningSceneInterfaceWrapper> PlanningSceneClass("PlanningSceneInterface");

  PlanningSceneClass.def("get_known_object_names", &PlanningSceneInterfaceWrapper::getKnownObjectNamesPython);
  PlanningSceneClass.def("get_known_object_names_in_roi", &PlanningSceneInterfaceWrapper::getKnownObjectNamesInROIPython);
}

}
}

BOOST_PYTHON_MODULE(_moveit_planning_scene_interface)
{
  using namespace moveit::planning_interface;
  wrap_planning_scene_interface();
}

/** @endcond */
