# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Sarah Elliott

# Functions for moving the base

import roslib
roslib.load_manifest('moveit_commander')
import rospy
import threading
import move_base_msgs.msg as mbm
import actionlib as al
import actionlib_msgs.msg as am
import geometry_msgs.msg as gm
import tf
import moveit_commander.exceptions as ex
import sensor_msgs.msg as sm
#import arm_navigation_msgs.msg as anm
import copy
#import sbpl_3dnav_planner.srv as sbpl
from math import sqrt

class Base():
    """
    Represents the Robot Base
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._js_lock = threading.Lock()
        self._js_counter = 0
        self._last_pose = None
        self._last_state = None
        self._sub = rospy.Subscriber('pose_gossip',
                                     gm.PoseWithCovarianceStamped,
                                     self._save_pose)
        #self._js_sub = rospy.Subscriber('joint_states', sm.JointState,
        #                                self._save_joint_state)
        #self._base_pose_srv = rospy.ServiceProxy('sbpl_full_body_planning/'
        #                                         'find_base_poses',
        #                                         sbpl.GetBasePoses)
        self._ac = al.SimpleActionClient('move_base', mbm.MoveBaseAction)
        rospy.loginfo("Waiting for move base action server...")
        self._ac.wait_for_server()
        rospy.loginfo("Move base action server ready")

    def move_to(self, x, y, theta, relative_position = False):
        """
        Moves base to a 2d pose specified in the map frame.  Possible outcomes:

        * The function returns successfully.  In this case, the robot is guaranteed to be at the goal pose.
        * An ActionFailed exception is thrown if navigation fails.
        * The robot loops forever.  This should only happen in situations involving malicious humans.
        """

        goal = mbm.MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = '/map'

        if relative_position:
            current_x,current_y,current_theta = self.get_current_pose()
            x = current_x + x
            y = current_y + y
            theta = current_theta + theta
            
        goal.target_pose.pose = _to_pose(x, y, theta)

        rospy.loginfo("Sending base goal ({0}, {1}, {2}) and waiting for result".\
                      format(x, y, theta))
        self._ac.send_goal(goal)
        self._ac.wait_for_result()

        if self._ac.get_state() != am.GoalStatus.SUCCEEDED:
            result = {'0':'Status: PENDING',
                      '1':'Status: ACTIVE',
                      '2':'Status: PREEMPTED',
                      '3':'Status: SUCCEEDED',
                      '4':'Status: ABORTED',
                      '5':'Status: REJECTED',
                      '6':'Status: PREEMPTING',
                      '7':'Status: RECALLING',
                      '8':'Status: RECALLED',
                      '9':'Status: LOST'}.get(str(self._ac.get_state()), str(self._ac.get_state()))
            rospy.loginfo(result)
            #print "Error is " + str(self._ac.get_state())
            #raise ex.ActionFailedError()
            return False
        else:
            print "Success."
            return True

    def get_current_pose_stamped(self):
        """
        :returns: Robot pose as geometry_msgs.msg.PoseWithCovarianceStamped
        """
        while not rospy.is_shutdown():
            with self._lock:
                if self._last_pose is not None:
                    break
                print "Waiting for pose to be received"
            rospy.sleep(1.0)
        if not rospy.is_shutdown():
            with self._lock:
                return self._last_pose.pose

    def get_current_pose(self):
        """
        :returns: Robot pose, as a tuple (x, y, theta) in the map frame.
        """
        while not rospy.is_shutdown():
            with self._lock:
                if self._last_pose is not None:
                    break
                print "Waiting for pose to be received"
            rospy.sleep(1.0)
        if not rospy.is_shutdown():
            with self._lock:
                return (self._last_pose.pose.pose.position.x,
                        self._last_pose.pose.pose.position.y,
                        _yaw(self._last_pose.pose.pose.orientation))
    """
    def _save_joint_state(self, m):

        # No need to save this 100 times a second
        self._js_counter += 1
        if self._js_counter % 20 == 0:
            with self._lock:
                if self._last_pose is None:
                    return
                with self._js_lock:
                    self._last_state = anm.RobotState()
                    self._last_state.joint_state.position = m.position
                    self._last_state.joint_state.name = m.name
                    md = self._last_state.multi_dof_joint_state
                    md.frame_ids.append('map')
                    md.child_frame_ids.append('base_footprint')
                    md.poses.append(self._last_pose.pose.pose)
    """

    def _save_pose(self, m):
        with self._lock:
            self._last_pose = m

def _yaw(q):
    e = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    return e[2]

def _to_quaternion(yaw):
    return gm.Quaternion(*tf.transformations.quaternion_from_euler(0, 0, yaw))

def _to_pose(x, y, theta):
    return gm.Pose(gm.Point(x, y, 0), _to_quaternion(theta))

