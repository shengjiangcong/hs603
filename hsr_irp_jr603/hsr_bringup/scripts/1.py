#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
import random

REFERENCE_FRAME = 'world'

TABLE_GROUND = 0.25

print "============ Starting tutorial setup"
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('loop_test', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("arm")

# Give the scene a chance to catch up    
rospy.sleep(2)

# Give each of the scene objects a unique name
base_table_id = 'base_table'


# Remove leftover objects from a previous run
scene.remove_world_object(base_table_id)


# Set the dimensions of the scene objects [l, w, h]
base_table_size = [3, 3, 0.01]

# Add a base table to the scene
base_table_pose = PoseStamped()
base_table_pose.header.frame_id = REFERENCE_FRAME
base_table_pose.pose.position.x = 0.0
base_table_pose.pose.position.y = 0.0
#base_table_pose.pose.position.z = TABLE_GROUND - base_table_size[2] / 2.0

base_table_pose.pose.position.z = 0.7
base_table_pose.pose.orientation.w = 1.0
scene.add_box(base_table_id, base_table_pose, base_table_size)


# Give the scene a chance to catch up    
rospy.sleep(2)
group.set_max_velocity_scaling_factor(0.5)
while not rospy.core.is_shutdown():
	group.set_random_target()
	group.go(wait=True)
	rospy.sleep(1)
