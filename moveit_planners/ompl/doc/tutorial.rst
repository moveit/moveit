OMPL Interface Documentation for MoveIt!
========================================

OMPL Optimization Objective Tutorial
------------------------------------

Several planners that are part of the OMPL planning library are capable of optimizing for a specified optimization objective. This tutorial describes that steps that are needed to configure these objectives. The optimal planners that are currently exposed to MoveIt! are:

* geometric::RRTstar
* geometric::PRMstar

And the following optimization objectives are available:

* PathLengthOptimizationObjective (Default)
* MechanicalWorkOptimizationObjective
* MaximizeMinClearanceObjective
* StateCostIntegralObjective
* MinimaxObjective

The configuration of these optimization objectives can be done in the *ompl_planning.yaml*. A parameter with the name **optimization_objective** is added as a configuration parameter. The value of the parameter is set to be the name of the selected optimization objective. For example, to configure RRTstar to use the *MaximizeMinClearanceObjective*, the planner entry in the ompl_planning.yaml will look like::

	RRTstarkConfigDefault:
	    type: geometric::RRTstar
	    optimization_objective: MaximizeMinClearanceObjective
	    range: 0.0
	    goal_bias: 0.05
	    delay_collision_checking: 1

For more information on the OMPL optimal planners, the reader is referred to the
`OMPL - Optimal Planning documentation <http://ompl.kavrakilab.org/optimalPlanning.html>`_.


Source Install For OMPL Within Catkin Workspace
-----------------------------------------------

These instructions assume you are using catkin_tools. Clone the OMPL repos from either Bitbucket or Github:

.. code-block:: bash

    git clone https://github.com/ompl/ompl

Next manually add a package.xml as used in the ROS release wrapper for OMPL (modify ROS distro version as necessary)

.. code-block:: bash

    wget https://raw.githubusercontent.com/ros-gbp/ompl-release/debian/kinetic/xenial/ompl/package.xml

Now you should be able to build using regular ``catkin build``
