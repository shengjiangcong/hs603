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
from copy import deepcopy

#水平底座的高度（相对于机械臂坐标系）
height_shuipin_collision = 0.8

#收纳盒中心位置
box2_x = 0.3725
box2_y = -0.18
#第三个数据表示放置时候各自的高度//间隔0.11
redStore   = [box2_x - 0.0725 - 0.055, box2_y - 0.055, 0.98]
greenStore = [box2_x - 0.0725 - 0.055, box2_y + 0.055, 0.98]
blueStore  = [box2_x - 0.0725 + 0.055, box2_y - 0.055, 0.98]
blackStore = [box2_x - 0.0725 + 0.055, box2_y + 0.055, 0.98]

capture_point = [0.39464, 0.2451, 1.1839]
capture_quaternion = Quaternion(0.70711, 0.70711, 0, 0) # Quaternion(0, 0, 0, 1)
home_point = [0.28685, 0.05, 1.2327]

#拍照识别位置的六轴角度
capture_joint = [0.5559790730476379, -0.8841944336891174, 2.39966082572937, -0.0009109065285883844, 1.624731421470642, 0.5258713674545288]

#四种物品抓取时候的各自高度
pick_red_height   = 0.975
pick_green_height = 0.975
pick_blue_height  = 0.975
pick_black_height = 0.975

#抓取时候的准备高度
pick_prepare_height = 1.07

#放置时候的准备高度
place_prepare_height = 1.15

red_count   = 0
green_count = 0
blue_count  = 0
black_count = 0


coll_x = 0.4
coll_y = 0
coll_z = 1.5

io_serv = rospy.ServiceProxy('/set_robot_io', setRobotIo)

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

        self.arm.set_max_velocity_scaling_factor (1.0)
#############添加水平障碍############
        rospy.sleep(1)
        base_table_id = 'base_table'
        self.scene.remove_world_object(base_table_id)
        base_table_size = [3, 3, 0.01]
        base_table_pose = PoseStamped()
        base_table_pose.header.frame_id = self.reference_frame
        base_table_pose.pose.position.x = 0
        base_table_pose.pose.position.y = 0.0
        base_table_pose.pose.position.z = height_shuipin_collision#高度
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

##############添加中间障碍#############
        middle_collision_id = 'middle_collision'
        self.scene.remove_world_object(middle_collision_id)
        middle_collision_size = [0.22, 0.02, 0.22]
        middle_collision_pose = PoseStamped()
        middle_collision_pose.header.frame_id = self.reference_frame
        middle_collision_pose.pose.position.x = 0.405
        middle_collision_pose.pose.position.y = home_point[1]
        middle_collision_pose.pose.position.z = height_shuipin_collision + middle_collision_size[2]/2.0
        middle_collision_pose.pose.orientation.w = 1.0
        self.scene.add_box(middle_collision_id, middle_collision_pose, middle_collision_size)
        rospy.sleep(1)

##############添加附着障碍#############
        tool1_id = 'tool1'
        self.scene.remove_attached_object(self.end_effector_link, tool1_id)
        tool1_size = [0.052, 0.13, 0.003]
        tool1_pose = PoseStamped()
        tool1_pose.header.frame_id = self.end_effector_link
        tool1_pose.pose.position.x = 0.0
        tool1_pose.pose.position.y = 0.015
        tool1_pose.pose.position.z = 0.0
        tool1_pose.pose.orientation.w = 1.0
        self.scene.attach_box(self.end_effector_link, tool1_id, tool1_pose, tool1_size)
        rospy.sleep(1)

##############添加附着障碍#############
        tool2_id = 'tool2'
        self.scene.remove_attached_object(self.end_effector_link, tool2_id)
        tool2_size = [0.015, 0.015, 0.18]
        tool2_pose = PoseStamped()
        tool2_pose.header.frame_id = self.end_effector_link
        tool2_pose.pose.position.x = 0.0
        tool2_pose.pose.position.y = 0.07
        tool2_pose.pose.position.z = 0.04
        tool2_pose.pose.orientation.w = 1.0
        self.scene.attach_box(self.end_effector_link, tool2_id, tool2_pose, tool2_size)
        rospy.sleep(1)

##############添加附着障碍#############
        tool3_id = 'tool3'
        self.scene.remove_attached_object(self.end_effector_link, tool3_id)
        tool3_size = [0.09, 0.03, 0.026]
        tool3_pose = PoseStamped()
        tool3_pose.header.frame_id = self.end_effector_link
        tool3_pose.pose.position.x = 0.0
        tool3_pose.pose.position.y = -0.035
        tool3_pose.pose.position.z = 0.013
        tool3_pose.pose.orientation.w = 1.0
        self.scene.attach_box(self.end_effector_link, tool3_id, tool3_pose, tool3_size)
        rospy.sleep(1)


        #initialize arm position to home
        self.arm.set_named_target('home')
        self.arm.go()
        rospy.sleep(1)

        try:
            #a = rospy.ServiceProxy('/set_robot_io', setRobotIo)
            resp = io_serv(3, True, False)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        rospy.sleep(1)

    def addbox(self, size, pose, name):
        backward_wall_id = name
        self.scene.remove_world_object(backward_wall_id)
        backward_wall_size = size
        backward_wall_pose = PoseStamped()
        backward_wall_pose.header.frame_id = self.reference_frame
        backward_wall_pose.pose.position.x = pose[0]
        backward_wall_pose.pose.position.y = pose[1]
        backward_wall_pose.pose.position.z = pose[2]
        backward_wall_pose.pose.orientation.w = 1.0
        self.scene.add_box(backward_wall_id, backward_wall_pose, backward_wall_size)
        rospy.sleep(1)

    def addmukuai(self):
        rospy.sleep(0.3)
        tool4_id = 'tool4'
        #self.scene.remove_attached_object(self.end_effector_link, tool4_id)
        tool4_size = [0.03, 0.03, 0.03]
        tool4_pose = PoseStamped()
        tool4_pose.header.frame_id = self.end_effector_link
        tool4_pose.pose.position.x = 0.0
        tool4_pose.pose.position.y = 0.07
        tool4_pose.pose.position.z = 0.15
        tool4_pose.pose.orientation.w = 1.0
        self.scene.attach_box(self.end_effector_link, tool4_id, tool4_pose, tool4_size)
        rospy.sleep(1)

    def rmmukuai(self):
        rospy.sleep(1)
        tool4_id = 'tool4'
        self.scene.remove_attached_object(self.end_effector_link, tool4_id)
        rospy.sleep(1)

    def moveToHome(self):
        self.arm.set_named_target('home')
        self.arm.go()
        rospy.sleep(0.5)   

    def pick_test(self, x, y, z):
            waypoints = []  
            start_pose = self.arm.get_current_pose(self.end_effector_link).pose   
            wpose = deepcopy(start_pose)
            wpose.position.x = x
            wpose.position.y = y
            wpose.position.z = pick_prepare_height
            waypoints.append(deepcopy(wpose))
            wpose.position.z = z
            waypoints.append(deepcopy(wpose))

            fraction = 0.0   #路径规划覆盖率
            maxtries = 100   #最大尝试规划次数
            attempts = 0     #已经尝试规划次数

            self.arm.set_start_state_to_current_state()

            while fraction < 1.0 and attempts < maxtries:
                (plan, fraction) = self.arm.compute_cartesian_path (
                                    waypoints,   # waypoint poses，路点列表
                                    0.03,        # eef_step，终端步进值
                                    0.0,         # jump_threshold，跳跃阈值
                                    True)        # avoid_collisions，避障规划
                attempts += 1
                if attempts % 10 == 0:
                    rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                   
            # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
            if fraction == 1.0:
                #print "解算成功"
                #rospy.sleep(0.3)
                self.arm.execute(plan)
                rospy.sleep(0.5)
                #print "运动执行成功"
            else:
                rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

            #print "打开吸盘"
            self.io_control(True)
            
            #self.addmukuai()   
            rospy.sleep(0.5)
            return True


    def place_test(self, x, y, z, pick_x, pick_y):
            waypoints = []  
            start_pose = self.arm.get_current_pose(self.end_effector_link).pose   
            wpose = deepcopy(start_pose)
            wpose.position.x = pick_x
            wpose.position.y = pick_y
            wpose.position.z = pick_prepare_height
            waypoints.append(deepcopy(wpose))
            #wpose.position.x = home_point[0]
            wpose.position.y = home_point[1]+0.05
            wpose.position.z = home_point[2]
            waypoints.append(deepcopy(wpose))
            wpose.position.y = home_point[1]-0.05
            waypoints.append(deepcopy(wpose))
            wpose.position.x = x
            wpose.position.y = y
            wpose.position.z = place_prepare_height
            waypoints.append(deepcopy(wpose))
            wpose.position.x = x
            wpose.position.y = y
            wpose.position.z = z
            waypoints.append(deepcopy(wpose))


            fraction = 0.0   #路径规划覆盖率
            maxtries = 100   #最大尝试规划次数
            attempts = 0     #已经尝试规划次数

            self.arm.set_start_state_to_current_state()

            while fraction < 1.0 and attempts < maxtries:
                (plan, fraction) = self.arm.compute_cartesian_path (
                                    waypoints,   # waypoint poses，路点列表
                                    0.03,        # eef_step，终端步进值
                                    0.0,         # jump_threshold，跳跃阈值
                                    True)        # avoid_collisions，避障规划
                attempts += 1
                if attempts % 10 == 0:
                    rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                   
            # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
            if fraction == 1.0:
                #print "解算成功"
                #rospy.sleep(0.3)
                self.arm.execute(plan)
                rospy.sleep(0.5)
                #print "运动执行成功"
            else:
                rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

            #print "关闭吸盘"
            self.io_control(False)
            rospy.sleep(0.5)     
            #self.rmmukuai()   
            #rospy.sleep(1)
            return True

    def move_to_camera(self, place_x, place_y):
            waypoints = []  
            start_pose = self.arm.get_current_pose(self.end_effector_link).pose   
            wpose = deepcopy(start_pose)
            wpose.position.x = place_x
            wpose.position.y = place_y
            wpose.position.z = place_prepare_height
            waypoints.append(deepcopy(wpose))
            wpose.position.x = home_point[0]
            wpose.position.y = home_point[1]
            wpose.position.z = home_point[2]
            waypoints.append(deepcopy(wpose))
            wpose.position.x = capture_point[0]
            wpose.position.y = capture_point[1]
            wpose.position.z = capture_point[2]
            waypoints.append(deepcopy(wpose))

            fraction = 0.0   #路径规划覆盖率
            maxtries = 100   #最大尝试规划次数
            attempts = 0     #已经尝试规划次数

            self.arm.set_start_state_to_current_state()

            while fraction < 1.0 and attempts < maxtries:
                (plan, fraction) = self.arm.compute_cartesian_path (
                                    waypoints,   # waypoint poses，路点列表
                                    0.1,        # eef_step，终端步进值
                                    0.0,         # jump_threshold，跳跃阈值
                                    True)        # avoid_collisions，避障规划

                attempts += 1
                if attempts % 10 == 0:
                    rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                   
            # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
            if fraction == 1.0:
                #print "解算成功"
                #rospy.sleep(0.3)
                self.arm.execute(plan)
                rospy.sleep(0.5)
                #print "运动执行成功"
            else:
                rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  
            return True        

    def moveJoint(self, value):
        rospy.sleep(0.1)
        group_variable_values = self.arm.get_current_joint_values()
        group_variable_values[0] = value[0]
        group_variable_values[1] = value[1]
        group_variable_values[2] = value[2]
        group_variable_values[3] = value[3]
        group_variable_values[4] = value[4]
        group_variable_values[5] = value[5]
        #print group_variable_values
        self.arm.set_joint_value_target(group_variable_values)
        traj = self.arm.plan()
        self.arm.execute(traj)
        rospy.sleep(0.1)

    def io_control(self, value):
        try:
            #a = rospy.ServiceProxy('/set_robot_io', setRobotIo)
            resp = io_serv(4, value, False)
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
    #demo.moveJoint(capture_joint)
    #demo.shutdown()

##########添加两个盒子###################

    size1_L = [0.24, 0.01, 0.075]
    size1_R = [0.24, 0.01, 0.075]
    size1_F = [0.01, 0.24, 0.075]
    size1_B = [0.01, 0.24, 0.075]

    box1_x = 0.3925
    box1_y = 0.25

    pose1_L = [box1_x, box1_y-0.12, height_shuipin_collision + size1_L[2]/2.0]
    pose1_R = [box1_x, box1_y+0.12, height_shuipin_collision + size1_R[2]/2.0]
    pose1_F = [box1_x-0.12, box1_y, height_shuipin_collision + size1_F[2]/2.0]
    pose1_B = [box1_x+0.12, box1_y, height_shuipin_collision + size1_B[2]/2.0]
    demo.addbox(size1_L, pose1_L, 'box1_L')  
    demo.addbox(size1_R, pose1_R, 'box1_R') 
    demo.addbox(size1_F, pose1_F, 'box1_F') 
    demo.addbox(size1_B, pose1_B, 'box1_B')

    #box2_x = 0.3725
    #box2_y = -0.18

    size2_L = [0.235, 0.01, 0.095]
    size2_R = [0.235, 0.01, 0.095]
    size2_F = [0.01, 0.235, 0.095]
    size2_B = [0.01, 0.235, 0.095]
    size2_M1= [0.235, 0.01, 0.095]
    size2_M2= [0.01, 0.235, 0.095]

    pose2_L = [box2_x, box2_y-0.1175, height_shuipin_collision + size2_L[2]/2.0]
    pose2_R = [box2_x, box2_y+0.1175, height_shuipin_collision + size2_R[2]/2.0]
    pose2_F = [box2_x-0.1175, box2_y, height_shuipin_collision + size2_F[2]/2.0]
    pose2_B = [box2_x+0.1175, box2_y, height_shuipin_collision + size2_B[2]/2.0]
    pose2_M1= [box2_x, box2_y, height_shuipin_collision + size2_M1[2]/2.0]
    pose2_M2= [box2_x, box2_y, height_shuipin_collision + size2_M2[2]/2.0]
    demo.addbox(size2_L, pose2_L, 'box2_L')  
    demo.addbox(size2_R, pose2_R, 'box2_R') 
    demo.addbox(size2_F, pose2_F, 'box2_F') 
    demo.addbox(size2_B, pose2_B, 'box2_B')
    demo.addbox(size2_M1, pose2_M1, 'box2_M1') 
    demo.addbox(size2_M2, pose2_M2, 'box2_M2S') 
    # 相机拍照位置
    demo.moveJoint(capture_joint)
    rospy.sleep(1)

    while not rospy.is_shutdown():

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
            #demo.pick_test(x_value,  y_value, pick_red_height)
            if demo.pick_test(x_value,  y_value, pick_red_height) == True:
                demo.place_test(redStore[0], redStore[1], redStore[2] + (red_count%3)*0.03, x_value, y_value )
                red_count = red_count + 1
                demo.move_to_camera(redStore[0], redStore[1])
        elif len(response.greenObjList):
            x_value = response.greenObjList[0].position.y * reg_x[0] + reg_x[1]
            y_value = response.greenObjList[0].position.x * reg_y[0] + reg_y[1]
            print "Pick Position: %f, %f"%(x_value, y_value)
            if demo.pick_test(x_value,  y_value, pick_red_height) == True:
                demo.place_test(greenStore[0], greenStore[1], greenStore[2] + (green_count%3)*0.03, x_value, y_value )
                green_count = green_count + 1
                demo.move_to_camera(greenStore[0], greenStore[1])
        elif len(response.blueObjList):
            x_value = response.blueObjList[0].position.y * reg_x[0] + reg_x[1]
            y_value = response.blueObjList[0].position.x * reg_y[0] + reg_y[1]
            print "Pick Position: %f, %f"%(x_value, y_value)
            if demo.pick_test(x_value,  y_value, pick_red_height) == True:
                demo.place_test(blueStore[0], blueStore[1], blueStore[2] + (blue_count%3)*0.03, x_value, y_value )
                blue_count = blue_count + 1
                demo.move_to_camera(blueStore[0], blueStore[1])
        elif len(response.blackObjList):
            x_value = response.blackObjList[0].position.y * reg_x[0] + reg_x[1]
            y_value = response.blackObjList[0].position.x * reg_y[0] + reg_y[1]
            print "Pick Position: %f, %f"%(x_value, y_value)
            if demo.pick_test(x_value,  y_value, pick_red_height) == True:
                demo.place_test(blackStore[0], blackStore[1], blackStore[2] + (black_count%3)*0.03, x_value, y_value )
                black_count = black_count + 1
                demo.move_to_camera(blackStore[0], blackStore[1])


        rate.sleep()

    demo.moveToHome()   
    demo.shutdown()
