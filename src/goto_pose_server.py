#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf.transformations
from std_msgs.msg import Bool
from roboarm.srv import PoseRequest, PoseRequestResponse

def goto_pose_server(request):
	group.set_pose_target(request.pose)
	plan = group.plan()

	display_trajectory.trajectory_start = roboarm.get_current_state()
	display_trajectory.trajectory.append(plan)
	display_trajectory_pub.publish(display_trajectory)

	success = group.go(wait=True)
	response = Bool()
	response.data = success
	return PoseRequestResponse(response)

sys.argv.append('/joint_states:=/roboarm/joint_states')
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('goto_pose_server', anonymous=True)

roboarm = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander('roboarm')
rospy.sleep(5)

print 'The planning frame is', group.get_planning_frame()
print 'The end effector link is', group.get_end_effector_link()
print 'The group name is', roboarm.get_group_names()
print 'The current state is', roboarm.get_current_state()

display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory_pub = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)

rospy.Service('goto_pose_server', PoseRequest, goto_pose_server)

rospy.spin()







#Test values
#pose_target.orientation.x = -0.501634507333
#pose_target.orientation.y = 1.72876577259e-05
#pose_target.orientation.z = -4.46482869593e-05
#pose_target.orientation.w = 0.865079660355
#pose_target.position.x = 1.06955479166
#pose_target.position.y = 0.775824538778
#pose_target.position.z = 1.42802207074