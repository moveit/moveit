import roslib
roslib.load_manifest('moveit_commander')
import rospy
import os
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from object_recognition_msgs.msg import TableArray
from sensor_msgs.msg import JointState
from moveit_commander.object_recognition import ObjectRecognition
from moveit_commander.commander import MoveGroupCommander 
from moveit_commander.planning_scene_interface import PlanningSceneInterface
import math
import tf
from tf.transformations import euler_from_quaternion

class ObjectListener(object):

    def __init__(self, planning_scene, listener):
        self._planning_scene = planning_scene
        self._listener = listener

    def table_callback(self, msg):
        #print msg

        self._planning_scene.remove_simple_object('table', 'base_link')
        self._listener.waitForTransform("/head_camera_rgb_optical_frame", "/base_link", rospy.Time(0), rospy.Duration(5.0))
        old_pose = PoseStamped()
        for table in msg.tables:
            old_pose.header.frame_id = 'head_camera_rgb_optical_frame'

            old_pose.pose = table.pose.pose
            #old_pose.point.y = table.pose.pose.position.y
            #old_pose.point.z = table.pose.pose.position.z
            if old_pose.pose.position.z > 1.0:
                continue
            else:
                print msg
                new_pose = self._listener.transformPose('base_link', old_pose)       
                self._planning_scene.add_box('table', 'base_link', [0.085 + (table.y_min + table.y_max)/2.0, -(table.x_min 
                                             + table.x_max)/2.0, (new_pose.pose.position.z/2.0)], 
                                             [new_pose.pose.orientation.x, new_pose.pose.orientation.y, 
                                             new_pose.pose.orientation.z, new_pose.pose.orientation.w], 
                                             table.x_max - table.x_min, table.y_max - table.y_min, new_pose.pose.position.z)
        


if __name__ == "__main__":
    # initialize
    rospy.init_node('test_arm_control', anonymous=True)

    object_recognizer = ObjectRecognition()

    arm_mover = MoveGroupCommander('arm')
    arm_mover.go([0,0,0,0,0,0,0])
    os.system('rosrun gripper_control lcg_set_pos.py -n low_cost -f 5 -p 0.13')


    listener = tf.TransformListener()
    planning_scene = PlanningSceneInterface()
    object_listener = ObjectListener(planning_scene, listener)
    rospy.Subscriber("/table_array", TableArray, object_listener.table_callback)

    while True:

        arm_mover.go([0,0,0,0,0,0,0])
        print "Looking for objects."
        objects = object_recognizer.look_for_objects() 
        while not objects.recognized_objects.objects:
            objects = object_recognizer.look_for_objects()
        
        rate = rospy.Rate(10.0)

        listener.waitForTransform("/head_camera_rgb_optical_frame", "/base_link", rospy.Time(0), rospy.Duration(5.0))
        old_point = PointStamped()
        old_point.header.frame_id = 'head_camera_rgb_optical_frame'
        
        old_point.point.x = objects.recognized_objects.objects[0].pose.pose.pose.position.x
        old_point.point.y = objects.recognized_objects.objects[0].pose.pose.pose.position.y
        old_point.point.z = objects.recognized_objects.objects[0].pose.pose.pose.position.z
        
        new_point = listener.transformPoint('base_link', old_point) 
        arm_mover.set_pose_reference_frame('base_link')
        euler = euler_from_quaternion([0.94, -0.3, -0.035, 0.03])

        #go to pre-grasp pose 
        pose = [new_point.point.x, new_point.point.y, new_point.point.z + 0.45, euler[0], euler[1], euler[2]]
        print "I want to move to: %s" % (str(pose))
        arm_mover.set_pose_target(pose, "wrist_roll_link")
        if not arm_mover.go():
            continue
            
        rospy.sleep(1.0)

        #move down to grasp
        new_pose = pose
        new_pose[2] = new_pose[2] - 0.18
        new_pose[1] = new_pose[1] + 0.02
        new_pose[0] = new_pose[0] + 0.02
        arm_mover.set_pose_target(new_pose, "wrist_roll_link")
        if not arm_mover.go():
            continue

        #hack to close gripper because pr2_controllers_msgs is nto catkinized
        os.system('rosrun gripper_control lcg_set_pos.py -n low_cost -f 5 -p 0.0')

        #move up post-grasp
        new_pose[2] = new_pose[2] + 0.10
        arm_mover.set_pose_target(new_pose, "wrist_roll_link")
        arm_mover.go()

        #"away" pose
        arm_mover.go([-0.6037502602262427, -1.1893971227100977, 0.8038406237070599, 0.3173183725777552, -1.7711143644272798, -0.03286443526651045, -0.05542899266273116])

        rospy.sleep(1.0)

        #put object down
        new_pose[2] = new_pose[2] - 0.10
        arm_mover.set_pose_target(new_pose, "wrist_roll_link")
        arm_mover.go()

        #gripper hack
        os.system('rosrun gripper_control lcg_set_pos.py -n low_cost -f 5 -p 0.13')

        #up twelve cm
        new_pose[2] = new_pose[2] + 0.12
        arm_mover.set_pose_target(new_pose, "wrist_roll_link")
        arm_mover.go()
        
        #home position  
        arm_mover.go([0,0,0,0,0,0,0])

        

        rospy.sleep(2.0)
