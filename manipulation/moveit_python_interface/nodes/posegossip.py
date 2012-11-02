#!/usr/bin/env python


"""To test in simulation:
$ roscore
$ roslaunch pr2_gazebo pr2_empty_world.launch
$ sudo apt-get install ros-electric-pr2-navigation-apps
$ rospack profile
$ roslaunch pr2_2dnav pr2_2dnav.launch
$ roscd map_server
$ ./bin/map_server test/testmap.yaml
$ rviz # ask ben to set up a bunch of crap
set 2d pose estimate
now pose gossip show be spitting out the pose. verify with
$ rostopic echo pose_gossip

"""


from copy import deepcopy
from threading import Lock
from time import sleep

import roslib; roslib.load_manifest('moveit_python_interface')

from geometry_msgs.msg import PoseWithCovarianceStamped
import rospy
from rospy import loginfo


NODE_NAME = 'amcl_pose_gossip'
SUB_TOPIC = 'amcl_pose'
PUB_TOPIC = 'pose_gossip'
PUB_RATE = 4

pose_data = None
data_lock = Lock()


def pose_callback(data):
    global pose_data
    data_lock.acquire()
    try:
        pose_data = deepcopy(data)
    finally:
        data_lock.release()

def main():
    loginfo("%s starting up." % NODE_NAME)
    rospy.init_node(NODE_NAME)
    sleep(1)
    
    rospy.Subscriber(SUB_TOPIC, PoseWithCovarianceStamped, pose_callback)
    pose_gossip = rospy.Publisher(PUB_TOPIC, PoseWithCovarianceStamped)
    loginfo("%s repeating what it hears on '%s'." % (NODE_NAME, SUB_TOPIC))
    
    rate = rospy.Rate(PUB_RATE)
    while not rospy.is_shutdown():
        data_lock.acquire()
        try:
            if pose_data is None:
                loginfo("%s not publishing because it hasn't heard any data "
                        "yet." % NODE_NAME)
            else:
                pose_gossip.publish(pose_data)
        finally:
            data_lock.release()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
