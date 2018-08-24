#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
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
#  * Neither the name of SRI International nor the names of its
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
# Author: Acorn Pooley

## BEGIN_SUB_TUTORIAL imports
##
## To use the python interface to move_group, import the moveit_commander
## module.  We also import rospy and some messages that we will use.
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from geometry_msgs.msg import Pose
from std_msgs.msg import String

rand_pose = Pose()

def move_group_python_interface_tutorial():
	global rand_pose
	
	print "============ Starting manual control node"
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('manual_control_ur5_node',
				anonymous=True)

	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()
	group = moveit_commander.MoveGroupCommander("manipulator")
	#group = moveit_commander.MoveGroupCommander("my_ur5_arm")

	display_trajectory_publisher = rospy.Publisher(
							'/move_group/display_planned_path',
							moveit_msgs.msg.DisplayTrajectory,
							queue_size=20)

	print "============ Waiting for RVIZ..."
	#rospy.sleep(10)
	print "============ Starting controls "
	
	'''
	## Getting Basic Information
	## ^^^^^^^^^^^^^^^^^^^^^^^^^
	##
	## We can get the name of the reference frame for this robot
	print "============ Reference frame: %s" % group.get_planning_frame()

	## We can also print the name of the end-effector link for this group
	print "============ Reference frame: %s" % group.get_end_effector_link()
	'''

	## We can get a list of all the groups in the robot
	print "============ Robot Groups:"
	print robot.get_group_names()

	## Sometimes for debugging it is useful to print the entire state of the robot.
	#print "============ Printing robot state"
	#print robot.get_current_state()
	#print "============"

	print "============ Printing robot pose"
	print group.get_current_pose()
	print "============"

	## Generating Random pose
	## ^^^^^^^^^^^^^^^^^^^^^^^
	print "============ Random Pose"
	#del rand_pose
	rand_pose = group.get_random_pose()
	print rand_pose
	print "============"

	pose_target = Pose()

	pose_target.position.x = 0.0751597997494
	pose_target.position.y = 0.10915
	pose_target.position.z = 0.482745886493
	pose_target.orientation.x = 0.0
	pose_target.orientation.y = -0.707388269171
	pose_target.orientation.z = 0.0
	pose_target.orientation.w = 0.706825181102
	group.set_pose_target(pose_target)
	'''
	pose_target.position.x = rand_pose.pose.position.x
	pose_target.position.y = rand_pose.pose.position.x
	pose_target.position.z = rand_pose.pose.position.x
	pose_target.orientation.x = rand_pose.pose.orientation.x
	pose_target.orientation.y = rand_pose.pose.orientation.y
	pose_target.orientation.z = rand_pose.pose.orientation.z
	pose_target.orientation.w = rand_pose.pose.orientation.w
	group.set_pose_target(pose_target)
	'''
	
	## Visualizing generated robot pose
	plan = group.go(wait=True)
	# Calling `stop()` ensures that there is no residual movement
	group.stop()
	print "============ Waiting while RVIZ displays plan..."
	rospy.sleep(5)


	## Moving to a pose goal
	## ^^^^^^^^^^^^^^^^^^^^^
	#group.go(wait=True)

	group.clear_pose_targets()

	## Then, we will get the current set of joint values for the group
	#group_variable_values = group.get_current_pose()
	#print "============ Current Pose: ", group_variable_values
	
	'''
	box_pose = geometry_msgs.msg.PoseStamped()
	box_pose.header.frame_id = robot.get_planning_frame()
	box_pose.pose.orientation.w = 1.0
	box_name = "box"
	scene.add_box(box_name, box_pose, size=(0.5, 0.5, 0.5))
	

	# If we exited the while loop without returning then we timed out
	return False


	## Adding/Removing Objects and Attaching/Detaching Objects
	## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	## First, we will define the collision object message
	collision_object = moveit_msgs.msg.CollisionObject(box_name)
	'''

	## When finished shut down moveit_commander.
	moveit_commander.roscpp_shutdown()
	print "============ STOPPING"


if __name__=='__main__':
  try:
	move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
	pass

'''
up:
  position: 
    x: 0.0751597997494
    y: 0.19145
    z: 0.482745886493
  orientation: 
    x: -0.499800878669
    y: -0.500199042064
    z: -0.500199042061
    w: 0.499800878672




'''

