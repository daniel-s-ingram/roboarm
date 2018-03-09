#!/usr/bin/env python
import rospy
from roboarm.srv import PoseRequest, PoseRequestResponse
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
from math import pi

def set_pose(pose, x, y, z, roll, pitch, yaw):
	q = quaternion_from_euler(roll, pitch, yaw)
	pose.orientation.x = q[0]
	pose.orientation.y = q[1]
	pose.orientation.z = q[2]
	pose.orientation.w = q[3]
	pose.position.x = x
	pose.position.y = y
	pose.position.z = z

rospy.init_node('pose_request_client', anonymous=True)
rospy.wait_for_service('goto_pose_server')

goto_pose = rospy.ServiceProxy('goto_pose_server', PoseRequest)
gripper_pub = rospy.Publisher('/roboarm/gripper_controller/command', Float64, queue_size=10, latch=True)

gripper_msg = Float64()
gripper_msg.data = 0.3

pose = Pose()
set_pose(pose, 1.07, 0.78, 0.25, pi, 0, 0)
response = goto_pose(pose)
if response.result:
	gripper_pub.publish(gripper_msg)

rospy.sleep(1)

set_pose(pose, 0, 1, 1.2, pi, 0, 0)
response = goto_pose(pose)

set_pose(pose, -1, 1, 1.1, pi, 0, 0)
response = goto_pose(pose)
if response.result:
	gripper_msg.data = 0
	gripper_pub.publish(gripper_msg)