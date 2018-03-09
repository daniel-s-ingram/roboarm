#!/usr/bin/env python
import rospy
from roboarm.srv import PoseRequest, PoseRequestResponse
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose


rospy.init_node('pose_request_client', anonymous=True)
rospy.wait_for_service('goto_pose_server')

goto_pose = rospy.ServiceProxy('goto_pose_server', PoseRequest)
gripper_pub = rospy.Publisher('/roboarm/gripper_controller/command', Float64, queue_size=10)

gripper_msg = Float64()
gripper_msg.data = 0.3

pose = Pose()
pose.orientation.x = -0.501634507333
pose.orientation.y = 1.72876577259e-05
pose.orientation.z = -4.46482869593e-05
pose.orientation.w = 0.865079660355
pose.position.x = 1.06955479166
pose.position.y = 0.775824538778
pose.position.z = 1.42802207074

goto_pose(pose)

gripper_pub.publish(gripper_msg, latch=True)
