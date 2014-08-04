#!/usr/bin/env python

import xml.dom.minidom
from operator import add


import rospy
import roslib
import numpy
import time
import tf
from std_msgs.msg import Empty, String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped

def signedSquare(val):
  if val > 0:
    sign = 1
  else:
    sign = -1
  return val * val * sign


# classes to use joystick of xbox, ps3(wired) and ps3(wireless).

class JoyStatus():
    def __init__(self):
        self.center = False
        self.select = False
        self.start = False
        self.L3 = False
        self.R3 = False
        self.square = False
        self.up = False
        self.down = False
        self.left = False
        self.right = False
        self.triangle = False
        self.cross = False
        self.circle = False
        self.L1 = False
        self.R1 = False
        self.L2 = False
        self.R2 = False
        self.left_analog_x = 0.0
        self.left_analog_y = 0.0
        self.right_analog_x = 0.0
        self.right_analog_y = 0.0

class XBoxStatus(JoyStatus):
    def __init__(self, msg):
        JoyStatus.__init__(self)
        if msg.buttons[8] == 1:
            self.center = True
        else:
            self.center = False
        if msg.buttons[6] == 1:
            self.select = True
        else:
            self.select = False
        if msg.buttons[7] == 1:
            self.start = True
        else:
            self.start = False
        if msg.buttons[9] == 1:
            self.L3 = True
        else:
            self.L3 = False
        if msg.buttons[10] == 1:
            self.R3 = True
        else:
            self.R3 = False
        if msg.buttons[2] == 1:
            self.square = True
        else:
            self.square = False
        if msg.buttons[1] == 1:
            self.circle = True
        else:
            self.circle = False
        if msg.axes[7] > 0.1:
            self.up = True
        else:
            self.up = False
        if msg.axes[7] < -0.1:
            self.down = True
        else:
            self.down = False
        if msg.axes[6] > 0.1:
            self.left = True
        else:
            self.left = False
        if msg.axes[6] < -0.1:
            self.right = True
        else:
            self.right = False
        if msg.buttons[3] == 1:
            self.triangle = True
        else:
            self.triangle = False
        if msg.buttons[0] == 1:
            self.cross = True
        else:
            self.cross = False
        if msg.buttons[4] == 1:
            self.L1 = True
        else:
            self.L1 = False
        if msg.buttons[5] == 1:
            self.R1 = True
        else:
            self.R1 = False
        if msg.axes[2] < -0.5:
            self.L2 = True
        else:
            self.L2 = False
        if msg.axes[5] < -0.5:
            self.R2 = True
        else:
            self.R2 = False
        self.left_analog_x = msg.axes[0]
        self.left_analog_y = msg.axes[1]
        self.right_analog_x = msg.axes[3]
        self.right_analog_y = msg.axes[4]
        self.orig_msg = msg

class PS3Status(JoyStatus):
    def __init__(self, msg):
        JoyStatus.__init__(self)
        # creating from sensor_msgs/Joy
        if msg.buttons[16] == 1:
            self.center = True
        else:
            self.center = False
        if msg.buttons[0] == 1:
            self.select = True
        else:
            self.select = False
        if msg.buttons[3] == 1:
            self.start = True
        else:
            self.start = False
        if msg.buttons[1] == 1:
            self.L3 = True
        else:
            self.L3 = False
        if msg.buttons[2] == 1:
            self.R3 = True
        else:
            self.R3 = False
        if msg.axes[15] < 0:
            self.square = True
        else:
            self.square = False
        if msg.axes[4] < 0:
            self.up = True
        else:
            self.up = False
        if msg.axes[6] < 0:
            self.down = True
        else:
            self.down = False
        if msg.axes[7] < 0:
            self.left = True
        else:
            self.left = False
        if msg.axes[5] < 0:
            self.right = True
        else:
            self.right = False
        if msg.axes[12] < 0:
            self.triangle = True
        else:
            self.triangle = False
        if msg.axes[14] < 0:
            self.cross = True
        else:
            self.cross = False
        if msg.axes[13] < 0:
            self.circle = True
        else:
            self.circle = False
        if msg.axes[10] < 0:
            self.L1 = True
        else:
            self.L1 = False
        if msg.axes[11] < 0:
            self.R1 = True
        else:
            self.R1 = False
        if msg.axes[8] < 0:
            self.L2 = True
        else:
            self.L2 = False
        if msg.axes[9] < 0:
            self.R2 = True
        else:
            self.R2 = False
        self.left_analog_x = msg.axes[0]
        self.left_analog_y = msg.axes[1]
        self.right_analog_x = msg.axes[2]
        self.right_analog_y = msg.axes[3]
        self.orig_msg = msg

class PS3WiredStatus(JoyStatus):
    def __init__(self, msg):
        JoyStatus.__init__(self)
        # creating from sensor_msgs/Joy
        if msg.buttons[16] == 1:
            self.center = True
        else:
            self.center = False
        if msg.buttons[0] == 1:
            self.select = True
        else:
            self.select = False
        if msg.buttons[3] == 1:
            self.start = True
        else:
            self.start = False
        if msg.buttons[1] == 1:
            self.L3 = True
        else:
            self.L3 = False
        if msg.buttons[2] == 1:
            self.R3 = True
        else:
            self.R3 = False
        if msg.buttons[15] == 1:
            self.square = True
        else:
            self.square = False
        if msg.buttons[4] == 1:
            self.up = True
        else:
            self.up = False
        if msg.buttons[6] == 1:
            self.down = True
        else:
            self.down = False
        if msg.buttons[7] == 1:
            self.left = True
        else:
            self.left = False
        if msg.buttons[5] == 1:
            self.right = True
        else:
            self.right = False
        if msg.buttons[12] == 1:
            self.triangle = True
        else:
            self.triangle = False
        if msg.buttons[14] == 1:
            self.cross = True
        else:
            self.cross = False
        if msg.buttons[13] == 1:
            self.circle = True
        else:
            self.circle = False
        if msg.buttons[10] == 1:
            self.L1 = True
        else:
            self.L1 = False
        if msg.buttons[11] == 1:
            self.R1 = True
        else:
            self.R1 = False
        if msg.buttons[8] == 1:
            self.L2 = True
        else:
            self.L2 = False
        if msg.buttons[9] == 1:
            self.R2 = True
        else:
            self.R2 = False
        self.left_analog_x = msg.axes[0]
        self.left_analog_y = msg.axes[1]
        self.right_analog_x = msg.axes[2]
        self.right_analog_y = msg.axes[3]
        self.orig_msg = msg

class StatusHistory():
  def __init__(self, max_length=10):
    self.max_length = max_length
    self.buffer = []
  def add(self, status):
    self.buffer.append(status)
    if len(self.buffer) > self.max_length:
      self.buffer = self.buffer[1:self.max_length+1]
  def all(self, proc):
    for status in self.buffer:
      if not proc(status):
        return False
    return True
  def latest(self):
    if len(self.buffer) > 0:
      return self.buffer[-1]
    else:
      return None
  def length(self):
    return len(self.buffer)
  def new(self, status, attr):
    if len(self.buffer) == 0:
      return getattr(status, attr)
    else:
      return getattr(status, attr) and not getattr(self.latest(), attr)
    
class MoveitJoy:
    def parseSRDF(self):
        # key := planning group name
        # value := list of the topic names
        planning_groups = {}
        srdf = rospy.get_param("/robot_description_semantic")
        # 1. get all the planning groups
        #   lookup all the <group> tags
        # 2. about for each <group> tag, find end effector
        dom = xml.dom.minidom.parseString(srdf)
        groups = dom.getElementsByTagName("group")
        groups_has_subgroups = []
        virtual_joints = dom.getElementsByTagName("virtual_joint")
        for group in groups:
            name = group.getAttribute("name")
            # check <chain> and <joint> tag
            # if it has <subgroup> tag, processing will be postponed
            # until all other groups finish to be processed
            chain = group.getElementsByTagName("chain")
            joint = group.getElementsByTagName("joint")
            subgroup = group.getElementsByTagName("group")
            if len(chain) > 0:
                tip_links = [c.getAttribute("tip_link") for c in chain]
                planning_groups[name] = ["/rviz/moveit/move_marker/goal_" + l 
                                         for l in tip_links]
            elif len(joint) > 0:
                joint_names = [j.getAttribute("name") for j in joint]
                # suppose joint is virtual_join and lookup child_link
                child_links = [[vj.getAttribute("child_link") for vj in virtual_joints if vj.getAttribute("name") == j]
                               for j in joint_names]
                planning_groups[name] = ["/rviz/moveit/move_marker/goal_" + l[0]
                                         for l in child_links if len(l) > 0]
            elif len(subgroup) > 0:       
                groups_has_subgroups.append(group)   #process after
            else:
                pass
        for group in groups_has_subgroups:
            subgroups = group.getElementsByTagName("group")
            name = group.getAttribute("name")
            all_topics = [planning_groups[subgroup.getAttribute("name")]
                          for subgroup in subgroups 
                          if planning_groups.has_key(subgroup.getAttribute("name"))]
            planning_groups[name] = reduce(add, all_topics)
        for name in planning_groups.keys():
            if len(planning_groups[name]) == 0:
                del planning_groups[name]
            else:
                print name, planning_groups[name]
        self.planning_groups = planning_groups
        self.planning_groups_keys = planning_groups.keys()   #we'd like to store the 'order'
    def __init__(self):
        self.prev_time = rospy.Time.now()
        self.counter = 0
        self.history = StatusHistory(max_length=10)
        self.pre_pose = PoseStamped()
        self.pre_pose.pose.orientation.w = 1
        self.current_planning_group_index = 0
        self.current_eef_index = 0
        self.parseSRDF()
        self.plan_group_pub = rospy.Publisher('/rviz/moveit/select_planning_group', String)
        self.updatePlanningGroup(0)
        self.updatePoseTopic(0)
        self.joy_pose_pub = rospy.Publisher("/joy_pose", PoseStamped)
        self.frame_id = rospy.get_param("~frame_id", "/odom_combined")
        self.plan_pub = rospy.Publisher("/rviz/moveit/plan", Empty)
        self.execute_pub = rospy.Publisher("/rviz/moveit/execute", Empty)
        self.update_start_state_pub = rospy.Publisher("/rviz/moveit/update_start_state", Empty)
        self.update_goal_state_pub = rospy.Publisher("/rviz/moveit/update_goal_state", Empty)
        self.sub = rospy.Subscriber("/joy", Joy, self.joyCB, queue_size=1)
    def updatePlanningGroup(self, next_index):
        if next_index >= len(self.planning_groups_keys):
            self.current_planning_group_index = 0
        elif next_index < 0:
            self.current_planning_group_index = len(self.planning_groups_keys) - 1
        else:
            self.current_planning_group_index = next_index
        next_planning_group = self.planning_groups_keys[self.current_planning_group_index]
        rospy.loginfo("change planning group to " + next_planning_group)
        self.plan_group_pub.publish(next_planning_group)
    def updatePoseTopic(self, next_index):
        planning_group = self.planning_groups_keys[self.current_planning_group_index]
        topics = self.planning_groups[planning_group]
        if next_index >= len(topics):
            self.current_eef_index = 0
        elif next_index < 0:
            self.current_eef_index = len(topics) - 1
        else:
            self.current_eef_index = next_index
        next_topic = topics[self.current_eef_index]
        rospy.loginfo("change planning eef to " + next_topic)
        self.pose_pub = rospy.Publisher(next_topic, PoseStamped)
    def joyCB(self, msg):
        if len(msg.axes) == 27 and len(msg.buttons) == 19:
            status = PS3WiredStatus(msg)
        elif len(msg.axes) == 8 and len(msg.buttons) == 11:
            status = XBoxStatus(msg)
        elif len(msg.axes) == 20 and len(msg.buttons) == 17:
            status = PS3Status(msg)
        else:
            raise Exception("unknown joystick")
        self.run(status)
        self.history.add(status)
    def run(self, status):
        pre_pose = self.pre_pose
        new_pose = PoseStamped()
        new_pose.header.frame_id = self.frame_id
        new_pose.header.stamp = rospy.Time(0.0)
        # move in local
        if status.square:
            scale = 10.0
        else:
            dist = status.left_analog_y * status.left_analog_y + status.left_analog_x * status.left_analog_x
            if dist > 0.9:
                scale = 20.0
            else:
                scale = 60.0
        x_diff = signedSquare(status.left_analog_y) / scale
        y_diff = signedSquare(status.left_analog_x) / scale
        # z
        if status.L2:
            z_diff = 0.005
        elif status.R2:
            z_diff = -0.005
        else:
            z_diff = 0.0
        if status.square:
            z_scale = 10.0
        elif self.history.all(lambda s: s.L2) or self.history.all(lambda s: s.R2):
            z_scale = 4.0
        else:
            z_scale = 2.0
        local_move = numpy.array((x_diff, y_diff,
                                  z_diff * z_scale, 
                                  1.0))
        q = numpy.array((pre_pose.pose.orientation.x,
                         pre_pose.pose.orientation.y,
                         pre_pose.pose.orientation.z,
                         pre_pose.pose.orientation.w))
        xyz_move = numpy.dot(tf.transformations.quaternion_matrix(q),
                         local_move)
        new_pose.pose.position.x = pre_pose.pose.position.x + xyz_move[0]
        new_pose.pose.position.y = pre_pose.pose.position.y + xyz_move[1]
        new_pose.pose.position.z = pre_pose.pose.position.z + xyz_move[2]
        roll = 0.0
        pitch = 0.0
        yaw = 0.0
        DTHETA = 0.02
        if status.L1:
            if status.square:
                yaw = yaw + DTHETA * 5
            elif self.history.all(lambda s: s.L1):
                yaw = yaw + DTHETA * 2
            else:
                yaw = yaw + DTHETA
        elif status.R1:
            if status.square:
                yaw = yaw - DTHETA * 5
            elif self.history.all(lambda s: s.R1):
                yaw = yaw - DTHETA * 2
            else:
                yaw = yaw - DTHETA
        if status.up:
            if status.square:
                pitch = pitch + DTHETA * 5
            elif self.history.all(lambda s: s.up):
                pitch = pitch + DTHETA * 2
            else:
                pitch = pitch + DTHETA
        elif status.down:
            if status.square:
                pitch = pitch - DTHETA * 5
            elif self.history.all(lambda s: s.down):
                pitch = pitch - DTHETA * 2
            else:
                pitch = pitch - DTHETA
        if status.right:
            if status.square:
                roll = roll + DTHETA * 5
            elif self.history.all(lambda s: s.right):
                roll = roll + DTHETA * 2
            else:
                roll = roll + DTHETA
        elif status.left:
            if status.square:
                roll = roll - DTHETA * 5
            elif self.history.all(lambda s: s.left):
                roll = roll - DTHETA * 2
            else:
                roll = roll - DTHETA
        diff_q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        new_q = tf.transformations.quaternion_multiply(q, diff_q)
        new_pose.pose.orientation.x = new_q[0]
        new_pose.pose.orientation.y = new_q[1]
        new_pose.pose.orientation.z = new_q[2]
        new_pose.pose.orientation.w = new_q[3]

        now = rospy.Time.from_sec(time.time())
        # placement.time_from_start = now - self.prev_time
        if (now - self.prev_time).to_sec() > 1 / 30.0:
            # rospy.loginfo(new_pose)
            self.pose_pub.publish(new_pose)
            self.joy_pose_pub.publish(new_pose)
            self.prev_time = now
        if self.history.new(status, "triangle"):   #increment planning group
            self.updatePlanningGroup(self.current_planning_group_index + 1)
            self.current_eef_index = 0    # force to reset
            self.updatePoseTopic(self.current_eef_index)
        elif self.history.new(status, "cross"):   #decrement planning group
            self.updatePlanningGroup(self.current_planning_group_index - 1)
            self.current_eef_index = 0    # force to reset
            self.updatePoseTopic(self.current_eef_index)
        elif self.history.new(status, "select"):
            self.updatePoseTopic(self.current_eef_index + 1)
        elif self.history.new(status, "start"):
            self.updatePoseTopic(self.current_eef_index - 1)
        elif self.history.new(status, "square"):   #plan
            rospy.loginfo("Plan")
            self.plan_pub.publish(Empty())
        elif self.history.new(status, "circle"):   #execute
            rospy.loginfo("Execute")
            self.execute_pub.publish(Empty())
        # sync start state to the real robot state
        self.counter = self.counter + 1
        if self.counter % 10:
            self.update_start_state_pub.publish(Empty())
        self.pre_pose = new_pose
        
    
if __name__ == "__main__":
    rospy.init_node("moveit_joy")
    app = MoveitJoy()
    rospy.spin()
