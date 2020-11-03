#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
import time

from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from object_color_detector.srv import *

redStore   = [0.3154, -0.2254]
greenStore = [0.3154, -0.3808]
blueStore  = [0.1391, -0.2254]

capture_point = Point(0.28, 0, 1.23)
capture_quaternion = Quaternion(0.70711, 0.70711, 0, 0) # Quaternion(0, 0, 0, 1)

pick_red_height   = 0.8
pick_green_height = 0.8
pick_blue_height  = 0.8
pick_prepare_height = 1

place_prepare_height = 1

red_count   = 0
green_count = 0
blue_count  = 0

class ProbotSortingDemo:
    def __init__(self):
        # Initialize ros and moveit 
        moveit_commander.roscpp_initialize(sys.argv)
                
        # Initialize moveit commander
        self.arm = moveit_commander.MoveGroupCommander('arm')
        self.scene = moveit_commander.PlanningSceneInterface()
                
        # Initialize arm effector link
        self.end_effector_link = self.arm.get_end_effector_link()
                        
        # Initialize reference frame
        self.reference_frame = 'world'
        self.arm.set_pose_reference_frame(self.reference_frame)
                
        # Initialize moveit parameters
        self.arm.allow_replanning(True)
        self.arm.set_goal_position_tolerance(0.001)
        self.arm.set_goal_orientation_tolerance(0.001)

#############添加水平障碍############
        rospy.sleep(1)
        base_table_id = 'base_table'
        self.scene.remove_world_object(base_table_id)
        base_table_size = [3, 3, 0.01]
        base_table_pose = PoseStamped()
        base_table_pose.header.frame_id = self.reference_frame
        base_table_pose.pose.position.x = 0.0
        base_table_pose.pose.position.y = 0.0
        base_table_pose.pose.position.z = 0.7#高度
        base_table_pose.pose.orientation.w = 1.0
        self.scene.add_box(base_table_id, base_table_pose, base_table_size)
        rospy.sleep(1)

##############添加后侧障碍#############
        backward_wall_id = 'backward_wall'
        self.scene.remove_world_object(backward_wall_id)
        backward_wall_size = [0.01, 3, 3]
        backward_wall_pose = PoseStamped()
        backward_wall_pose.header.frame_id = self.reference_frame
        backward_wall_pose.pose.position.x = -0.3
        backward_wall_pose.pose.position.y = 0.0
        backward_wall_pose.pose.position.z = 0.4
        backward_wall_pose.pose.orientation.w = 1.0
        self.scene.add_box(backward_wall_id, backward_wall_pose, backward_wall_size)
        rospy.sleep(1)


        #initialize arm position to home
        self.arm.set_named_target('home')
        self.arm.go()
        rospy.sleep(1)

    def moveToHome(self):
        self.arm.set_named_target('home')
        self.arm.go()
        rospy.sleep(1)   

    def moveTo(self, x, y, z): 
        # Create pose data           
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        target_pose.pose.orientation = capture_quaternion
        
        # Set pick position 
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        
        traj = self.arm.plan()

        if len(traj.joint_trajectory.points) == 0:
            return False

        self.arm.execute(traj)
        
        return True
    
    def pick(self, x, y, z):
        if self.moveTo(x, y, pick_prepare_height) == True:
            print "Pick Once"
            self.moveTo(x, y, z)
            
            rospy.sleep(1)
            self.moveTo(x, y, pick_prepare_height)
            
            return True
        else:
            print "Can not pick"
            return False
        
    def place(self, x, y, z): 
        if self.moveTo(x, y, place_prepare_height) == True:
            print "Place Once"
            self.moveTo(x, y, z)

            rospy.sleep(1)
            self.moveTo(x, y, place_prepare_height)

        else:
            print "Can not place"

    def moveSingleJoint(self, index, value):
        group_variable_values = self.arm.get_current_joint_values()
        group_variable_values[index] = value
        print group_variable_values
        self.arm.set_joint_value_target(group_variable_values)
        traj = self.arm.plan()
        self.arm.execute(traj)

    def shutdown(self):
        # Exit
        print "The demo is shutting down."
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":

    time.sleep(3)

    rospy.init_node('probot_vision_sorting_demo')

    demo = ProbotSortingDemo()
    demo.moveToHome()   
    #demo.moveTo(capture_point.x, capture_point.y, capture_point.z)
    demo.moveSingleJoint(1, -0.3)
    

    #demo.moveToHome()   
    demo.shutdown()
