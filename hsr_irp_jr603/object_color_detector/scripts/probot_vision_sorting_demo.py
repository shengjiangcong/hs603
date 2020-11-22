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
from hsr_rosi_device.srv import *

redStore   = [0.105, -0.29, 0.92]
greenStore = [0.215, -0.29, 0.92]
blueStore  = [0.105, -0.40, 0.92]
blackStore  = [0.1391, -0.2254, 0.92]

capture_point = Point(0.38876, 0, 1.17)
capture_quaternion = Quaternion(0.70711, 0.70711, 0, 0) # Quaternion(0, 0, 0, 1)
#拍照的六轴角度
capture_joint = [0.97976, -0.96643, 2.6815, 0, 1.4258, 0.9799]
#存放的六轴角度
green_place = [[-0.91218, -1.11, 3.355, 0.0016, 0.895, -0.913], [-0.9119, -0.8434, 3.414, 0.002, 0.569, -0.913], [-0.912, -0.9477, 3.414, 0.002, 0.6769, -0.913], [-0.912, -1.026, 3.39, 0.0018, 0.772, -0.913]]

red_place = [[-0.91218, -1.11, 3.355, 0.0016, 0.895, -0.913], [-0.9119, -0.8434, 3.414, 0.002, 0.569, -0.913], [-0.912, -0.9477, 3.414, 0.002, 0.6769, -0.913], [-0.912, -1.026, 3.39, 0.0018, 0.772, -0.913]]

blue_place = [[-0.91218, -1.11, 3.355, 0.0016, 0.895, -0.913], [-0.9119, -0.8434, 3.414, 0.002, 0.569, -0.913], [-0.912, -0.9477, 3.414, 0.002, 0.6769, -0.913], [-0.912, -1.026, 3.39, 0.0018, 0.772, -0.913]]

black_place = [[-0.91218, -1.11, 3.355, 0.0016, 0.895, -0.913], [-0.9119, -0.8434, 3.414, 0.002, 0.569, -0.913], [-0.912, -0.9477, 3.414, 0.002, 0.6769, -0.913], [-0.912, -1.026, 3.39, 0.0018, 0.772, -0.913]]

pick_red_height   = 0.91
pick_green_height = 0.91
pick_blue_height  = 0.91
pick_black_height  = 0.91
pick_prepare_height = 1.00

place_prepare_height = 1.03

red_count   = 0
green_count = 0
blue_count  = 0
black_count = 0

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
        base_table_pose.pose.position.z = 0.86#高度
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
        point_num = len(traj.joint_trajectory.points)
        a = traj.joint_trajectory.points[point_num - 1].positions[0]#a表示目标位子的joint0的角度
        while a < 0:
              traj = self.arm.plan()
              point_num = len(traj.joint_trajectory.points)
              a = traj.joint_trajectory.points[point_num - 1].positions[0]#a表示目标位子的joint0的角度
        print a

        if len(traj.joint_trajectory.points) == 0:
            return False

        self.arm.execute(traj)
        
        return True

    def moveToplace(self, x, y, z): 
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
        point_num = len(traj.joint_trajectory.points)
        a = traj.joint_trajectory.points[point_num - 1].positions[0]#a表示目标位子的joint0的角度
        while a > 0:
              traj = self.arm.plan()
              point_num = len(traj.joint_trajectory.points)
              a = traj.joint_trajectory.points[point_num - 1].positions[0]#a表示目标位子的joint0的角度
        print a

        if len(traj.joint_trajectory.points) == 0:
            return False

        self.arm.execute(traj)
        
        return True
    
    def pick(self, x, y, z):
        if self.moveTo(x, y, pick_prepare_height) == True:
            print "Pick Once"
            self.moveTo(x, y, z)
            print "打开吸盘"
            self.io_control(True)
            
            
            rospy.sleep(1)
            self.moveTo(x, y, pick_prepare_height)
            
            return True
        else:
            print "Can not pick"
            return False
        
    def place(self, x, y, z): 
        if self.moveToplace(x, y, place_prepare_height) == True:
            print "Place Once"
            self.moveToplace(x, y, z)

            print "关闭吸盘"
            self.io_control(False)

            self.moveToplace(x, y, place_prepare_height)

        else:
            print "Can not place"

    def moveJoint(self, value):
        group_variable_values = self.arm.get_current_joint_values()
        group_variable_values[0] = value[0]
        group_variable_values[1] = value[1]
        group_variable_values[2] = value[2]
        group_variable_values[3] = value[3]
        group_variable_values[4] = value[4]
        group_variable_values[5] = value[5]
        print group_variable_values
        self.arm.set_joint_value_target(group_variable_values)
        traj = self.arm.plan()
        self.arm.execute(traj)
        rospy.sleep(1)

    def io_control(self, value):
        rospy.wait_for_service('/set_robot_io')
        try:
            a = rospy.ServiceProxy('/set_robot_io', setRobotIo)
            resp = a(1, value, False)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        rospy.sleep(1)
        try:
            a = rospy.ServiceProxy('/set_robot_io', setRobotIo)
            resp = a(2, value, False)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        rospy.sleep(1)

    def shutdown(self):
        # Exit
        print "The demo is shutting down."
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":

    time.sleep(3)

    rospy.init_node('probot_vision_sorting_demo')
    rate = rospy.Rate(10)

    reg_x = rospy.get_param('/image/reg_x')
    reg_y = rospy.get_param('/image/reg_y')

    print "Probot sorting demo start."
    demo = ProbotSortingDemo()
   # demo.moveJoint(red_place)
    #demo.shutdown()
    while not rospy.is_shutdown():
        # 相机拍照位置
        demo.moveJoint(capture_joint)

        # Get target
        rospy.wait_for_service('/object_detect')
        try:
            detect_object_service = rospy.ServiceProxy('/object_detect', DetectObjectSrv)
            response = detect_object_service(DetectObjectSrvRequest.ALL_OBJECT) 
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        if response.result is not DetectObjectSrvResponse.SUCCESS:
            rospy.loginfo("No objects detected, waiting detecting...")
            rate.sleep()
            continue
        
        rospy.loginfo("Get object position, Start pick and place.")

        # Pick and place bject
        if len(response.redObjList):
            x_value = response.redObjList[0].position.y * reg_x[0] + reg_x[1]
            y_value = response.redObjList[0].position.x * reg_y[0] + reg_y[1]
            print "Pick Position: %f, %f"%(x_value, y_value)
            if demo.pick(x_value,  y_value, pick_red_height) == True:
                demo.place(redStore[0], redStore[1], redStore[2] + red_count*0.03)
                red_count = red_count + 1
        elif len(response.greenObjList):
            x_value = response.greenObjList[0].position.y * reg_x[0] + reg_x[1]
            y_value = response.greenObjList[0].position.x * reg_y[0] + reg_y[1]
            print "Pick Position: %f, %f"%(x_value, y_value)
            if demo.pick(x_value,  y_value, pick_green_height) == True:
                demo.place(greenStore[0], greenStore[1], greenStore[2] + green_count*0.03)
                green_count = green_count + 1
        elif len(response.blueObjList):
            x_value = response.blueObjList[0].position.y * reg_x[0] + reg_x[1]
            y_value = response.blueObjList[0].position.x * reg_y[0] + reg_y[1]
            print "Pick Position: %f, %f"%(x_value, y_value)
            if demo.pick(x_value,  y_value, pick_blue_height) == True:
                demo.place(blueStore[0], blueStore[1], blueStore[2] + blue_count*0.03)
                blue_count = blue_count + 1
        elif len(response.blackObjList):
            x_value = response.blackObjList[0].position.y * reg_x[0] + reg_x[1]
            y_value = response.blackObjList[0].position.x * reg_y[0] + reg_y[1]
            print "Pick Position: %f, %f"%(x_value, y_value)
            if demo.pick(x_value,  y_value, pick_black_height) == True:
                demo.place(blackStore[0], blackStore[1], blackStore[2] + black_count*0.03)
                black_count = black_count + 1

        rate.sleep()

    demo.moveToHome()   
    demo.shutdown()
