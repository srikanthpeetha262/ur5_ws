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
	rospy.init_node('ur5_controller', anonymous=True)

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
	
	## List of all the groups in the robot
	print "============ Robot Groups:"
	print robot.get_group_names()

	print "============ Printing robot pose"
	print group.get_current_rpy()
	print group.get_current_joint_values()
	print group.get_active_joints()
	print "============"


	print "============ Waiting while RVIZ displays plan..."
	rospy.sleep(5)

	group.clear_pose_targets()

	moveit_commander.roscpp_shutdown()
	print "============ STOPPING"


if __name__=='__main__':
  try:
	move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
	pass


