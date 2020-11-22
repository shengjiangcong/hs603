#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2019 Wuhan PS-Micro Technology Co., Itd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy, sys
import moveit_commander
import numpy as np
import math
import yaml

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import CompressedImage
from object_color_detector.srv import *
from sklearn.linear_model import LinearRegression

# 定义标定位姿点
calibration_points_z = 0.776 #0.2
calibration_points = []
calibration_points.append(Point(0.30, 0,    calibration_points_z))
calibration_points.append(Point(0.30, 0.1,  calibration_points_z))
calibration_points.append(Point(0.35, 0.1, calibration_points_z))
calibration_points.append(Point(0.35, -0.1,  calibration_points_z))
calibration_points.append(Point(0.30, -0.1,    calibration_points_z))

# 定义拍照位姿
capture_point = Point(0.201164, 0.26694, 0.83)
capture_quaternion = Quaternion(0.70711, 0.70711, 0, 0) #Quaternion(0, 0, 0, 1)

# 初始化move_group的API
moveit_commander.roscpp_initialize(sys.argv)

# 初始化ROS节点
rospy.init_node('auto_calibration')
        
# 初始化需要使用move group控制的机械臂中的arm group
scene = moveit_commander.PlanningSceneInterface()
arm = moveit_commander.MoveGroupCommander('arm')
reference_frame = 'world'

# 设置目标位置所使用的参考坐标系
arm.set_pose_reference_frame(reference_frame)
        
# 当运动规划失败后，允许重新规划
arm.allow_replanning(True)

# 设置位置(单位：米)和姿态（单位：弧度）的允许误差
arm.set_goal_position_tolerance(0.0001)
arm.set_goal_orientation_tolerance(0.0001)

#############添加水平障碍############
rospy.sleep(1)
base_table_id = 'base_table'
scene.remove_world_object(base_table_id)
base_table_size = [3, 3, 0.01]
base_table_pose = PoseStamped()
base_table_pose.header.frame_id = reference_frame
base_table_pose.pose.position.x = 0.0
base_table_pose.pose.position.y = 0.0
base_table_pose.pose.position.z = 0.7#高度
base_table_pose.pose.orientation.w = 1.0
scene.add_box(base_table_id, base_table_pose, base_table_size)
rospy.sleep(1)

##############添加后侧障碍#############
backward_wall_id = 'backward_wall'
scene.remove_world_object(backward_wall_id)
backward_wall_size = [0.01, 3, 3]
backward_wall_pose = PoseStamped()
backward_wall_pose.header.frame_id = reference_frame
backward_wall_pose.pose.position.x = -0.3
backward_wall_pose.pose.position.y = 0.0
backward_wall_pose.pose.position.z = 0.4
backward_wall_pose.pose.orientation.w = 1.0
scene.add_box(backward_wall_id, backward_wall_pose, backward_wall_size)
rospy.sleep(1)





# 获取终端link的名称
end_effector_link = arm.get_end_effector_link()
print(end_effector_link)
                

# 控制机械臂先回到初始化位置
arm.set_named_target('home')
arm.go()
rospy.sleep(2)

# 设置机械臂工作空间中的目标位姿
target_pose = PoseStamped()
target_pose.header.frame_id = reference_frame
target_pose.header.stamp = rospy.Time.now()     
target_pose.pose.orientation = capture_quaternion

capture_pose = target_pose
capture_pose.pose.position = capture_point

# 设置图像识别为调试模式
#config_filename = rospy.get_param("~filename")

# 标定参数
regObjList  = []

image_params = rospy.get_param("/image")
success_count= 0

xarray = np.zeros(len(calibration_points))
yarray = np.zeros(len(calibration_points))
xc_array = np.zeros(len(calibration_points))
yc_array = np.zeros(len(calibration_points))


def moveTo(x, y, z): 
    # Create pose data           
    target_pose = PoseStamped()
    target_pose.header.frame_id = reference_frame
    target_pose.header.stamp = rospy.Time.now()     
    target_pose.pose.position.x = x
    target_pose.pose.position.y = y
    target_pose.pose.position.z = z
    target_pose.pose.orientation = capture_quaternion
    
    # Set pick position 
    arm.set_start_state_to_current_state()
    arm.set_pose_target(target_pose, end_effector_link)
    
    traj = arm.plan()
    arm.execute(traj)


moveTo(capture_point.x, capture_point.y, capture_point.z)
rospy.sleep(1)

# 控制机械臂回到初始化位置
#arm.set_named_target('home')
#arm.go()

# 关闭并退出moveit
moveit_commander.roscpp_shutdown()
moveit_commander.os._exit(0)

