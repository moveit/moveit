#!/usr/bin/env python

# Take joystick cmds. Republish them as TwistStamped for
# e.g. jogging.

import rospy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy

class joy_to_twist:

    def __init__(self):
        self.pub = rospy.Publisher('jog_arm_server/delta_jog_cmds', TwistStamped, queue_size=1)

    # Convert to TwistStamped and republish
    def callback(self, joy):

        ts = TwistStamped()

        ts.header.stamp = rospy.Time.now()

        ts.twist.linear.x = joy.axes[0]
        ts.twist.linear.y = joy.axes[1]
        ts.twist.linear.z = joy.axes[2]

        ts.twist.angular.x = joy.axes[3]
        ts.twist.angular.y = joy.axes[4]
        ts.twist.angular.z = joy.axes[5]

        self.pub.publish(ts)
        
    def republish(self):
        rospy.Subscriber("spacenav/joy", Joy, self.callback)

        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('joy_to_twist', anonymous=True)
    republisher = joy_to_twist()
    republisher.republish()