#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf.transformations

sys.argv.append('/joint_states:=/roboarm/joint_states')
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('roboarm_move_group', anonymous=True)

roboarm = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander('roboarm')

print 'The planning frame is', group.get_planning_frame()
print 'The end effector link is', group.get_end_effector_link()
print 'The group name is', roboarm.get_group_names()
print 'The current state is', roboarm.get_current_state()

display_trajectory_pub = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)
rospy.sleep(5) #Wait for rviz

pose_target = geometry_msgs.msg.Pose()

#quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)

#Test values
pose_target.orientation.x = -0.501634507333 #quaternion[0]
pose_target.orientation.y = 1.72876577259e-05 #quaternion[1]
pose_target.orientation.z = -4.46482869593e-05 #quaternion[2]
pose_target.orientation.w = 0.865079660355 #quaternion[3]

pose_target.position.x = 1.06955479166
pose_target.position.y = 0.775824538778
pose_target.position.z = 1.42802207074

group.set_pose_target(pose_target)

plan = group.plan()
rospy.sleep(5)

display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = roboarm.get_current_state()
display_trajectory.trajectory.append(plan)
display_trajectory_pub.publish(display_trajectory)

group.go(wait=True)