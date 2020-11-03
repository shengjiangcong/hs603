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
from hsr_rosi_device.srv import *

rospy.init_node('io_test')
rospy.loginfo("io_test")

rospy.wait_for_service('/set_robot_io')
try:
    a = rospy.ServiceProxy('/set_robot_io', setRobotIo)
    response = a(1, True, False)
except rospy.ServiceException, e:
    print "Service call failed: %s"%e
rospy.sleep(1)
try:
    a = rospy.ServiceProxy('/set_robot_io', setRobotIo)
    response = a(2, True, False)
except rospy.ServiceException, e:
    print "Service call failed: %s"%e

rospy.sleep(5)

try:
    a = rospy.ServiceProxy('/set_robot_io', setRobotIo)
    response = a(1, False, False)
except rospy.ServiceException, e:
    print "Service call failed: %s"%e
rospy.sleep(1)
try:
    a = rospy.ServiceProxy('/set_robot_io', setRobotIo)
    response = a(2, False, False)
except rospy.ServiceException, e:
    print "Service call failed: %s"%e


# 关闭并退出moveit
moveit_commander.roscpp_shutdown()
moveit_commander.os._exit(0)

