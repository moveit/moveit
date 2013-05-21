# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
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
# Author: Ioan Sucan, Sarah Elliott

from moveit_commander import MoveItCommanderException
from geometry_msgs.msg import Pose, PoseStamped, Transform, TransformStamped
from moveit_msgs.msg import RobotTrajectory, MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import rospy
import tf

def pose_to_list(pose_msg):
    pose = []
    pose.append(pose_msg.position.x)
    pose.append(pose_msg.position.y)
    pose.append(pose_msg.position.z) 
    pose.append(pose_msg.orientation.x)
    pose.append(pose_msg.orientation.y)
    pose.append(pose_msg.orientation.z)
    pose.append(pose_msg.orientation.w)
    return pose

def transform_to_list(trf_msg):
    trf = []
    trf.append(trf_msg.translation.x)
    trf.append(trf_msg.translation.y)
    trf.append(trf_msg.translation.z) 
    trf.append(trf_msg.rotation.x)
    trf.append(trf_msg.rotation.y)
    trf.append(trf_msg.rotation.z)
    trf.append(trf_msg.rotation.w)
    return trf

def list_to_pose_stamped(pose_list, target_frame):
    pose_msg = PoseStamped()
    if len(pose_list) == 7:
        pose_msg.pose.position.x = pose_list[0]
        pose_msg.pose.position.y = pose_list[1]
        pose_msg.pose.position.z = pose_list[2]
        pose_msg.pose.orientation.x = pose_list[3]
        pose_msg.pose.orientation.y = pose_list[4]
        pose_msg.pose.orientation.z = pose_list[5]
        pose_msg.pose.orientation.w = pose_list[6]
    elif len(pose_list) == 6: 
        pose_msg.pose.position.x = pose_list[0]
        pose_msg.pose.position.y = pose_list[1]
        pose_msg.pose.position.z = pose_list[2]
        q = tf.transformations.quaternion_from_euler(pose_list[3], pose_list[4], pose_list[5])
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]
    else:
        raise MoveItCommanderException("Expected either 6 or 7 elements in list: (x,y,z,r,p,y) or (x,y,z,qx,qy,qz,qw)")
    pose_msg.header.frame_id = target_frame
    pose_msg.header.stamp = rospy.Time.now()
    return pose_msg

def list_to_transform(trf_list):
    trf_msg = Transform()
    trf_msg.translation.x = trf_list[0]
    trf_msg.translation.y = trf_list[1]
    trf_msg.translation.z = trf_list[2]
    trf_msg.rotation.x = trf_list[3]
    trf_msg.rotation.y = trf_list[4]
    trf_msg.rotation.z = trf_list[5]
    trf_msg.rotation.w = trf_list[6]
    return trf_msg

def dict_to_trajectory(plan):
    plan_msg = RobotTrajectory()
    joint_traj = JointTrajectory()
    joint_traj.header.frame_id = plan["joint_trajectory"]["frame_id"]
    joint_traj.joint_names = plan["joint_trajectory"]["joint_names"]
    for point in plan["joint_trajectory"]["points"]:
        joint_traj.points.append(JointTrajectoryPoint(
                positions = point["positions"],
                velocities = point["velocities"],
                accelerations = point["accelerations"],
                time_from_start = point["time_from_start"]))
    multi_dof_joint_traj = MultiDOFJointTrajectory()
    multi_dof_joint_traj.header.frame_id = plan["multi_dof_joint_trajectory"]["frame_id"]
    multi_dof_joint_traj.joint_names = plan["multi_dof_joint_trajectory"]["joint_names"]
    for point in plan["multi_dof_joint_trajectory"]["points"]:
        multi_dof_joint_traj_point = MultiDOFJointTrajectoryPoint()
        for t in point["transforms"]:
            multi_dof_joint_traj_point.transforms.append(list_to_transform(t))
        multi_dof_joint_traj_point.time_from_start = point["time_from_start"]
        multi_dof_joint_traj.points.append(multi_dof_joint_traj_point)
    plan_msg.joint_trajectory = joint_traj
    plan_msg.multi_dof_joint_trajectory = multi_dof_joint_traj
    return plan_msg    

def trajectory_to_dict(plan_msg):
    plan = {}
    plan["joint_trajectory"] = {}
    plan["joint_trajectory"]["frame_id"] = plan_msg.joint_trajectory.header.frame_id
    plan["joint_trajectory"]["joint_names"] = plan_msg.joint_trajectory.joint_names
    joint_trajectory_points = []
    for p in plan_msg.joint_trajectory.points:
        point = {}
        point["positions"] = p.positions
        point["velocities"] = p.velocities
        point["accelerations"] = p.accelerations
        point["time_from_start"] = p.time_from_start
        joint_trajectory_points.append(point)
    plan["joint_trajectory"]["points"] = joint_trajectory_points

    plan["multi_dof_joint_trajectory"] = {}
    plan["multi_dof_joint_trajectory"]["frame_id"] = plan_msg.multi_dof_joint_trajectory.header.frame_id
    plan["multi_dof_joint_trajectory"]["joint_names"] = plan_msg.multi_dof_joint_trajectory.joint_names
    multi_dof_joint_trajectory_points = []
    for p in plan_msg.multi_dof_joint_trajectory.points:
        point = {}
        point["transforms"] = []
        for t in p.transforms:
            point["transforms"].append(transform_to_list(t))
        point["time_from_start"] = p.time_from_start
        multi_dof_joint_trajectory_points.append(point)
    plan["multi_dof_joint_trajectory"]["points"] = multi_dof_joint_trajectory_points
    return plan
