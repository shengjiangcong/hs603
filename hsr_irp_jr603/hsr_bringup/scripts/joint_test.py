#!/usr/bin/env python

import rospy, sys
import moveit_commander
import time

from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from probot_msgs.msg import DemoCtrl
from probot_msgs.msg import SetOutputIO
from object_color_detector.srv import *

DEMO_START = 1
DEMO_STOP  = 2
demoCtrlCode_ = DEMO_STOP
redBox   = [0.32, 0.19]
greenBox = [0.29, 0.19]
blueBox  = [0.26, 0.19]
blackBox = [0.23, 0.19]

class ProbotSortingDemo:
    def __init__(self):
        # Initialize ros and moveit 
        moveit_commander.roscpp_initialize(sys.argv)
                
        # Initialize moveit commander
        self.arm = moveit_commander.MoveGroupCommander('manipulator')
                
        # Initialize arm effector link
        self.end_effector_link = self.arm.get_end_effector_link()
                        
        # Initialize reference frame
        self.reference_frame = 'base_link'
        self.arm.set_pose_reference_frame(self.reference_frame)
                
        # Initialize moveit parameters
        self.arm.allow_replanning(True)
        self.arm.set_goal_position_tolerance(0.001)
        self.arm.set_goal_orientation_tolerance(0.001)

        # Initialize IO
        self.ioPub = rospy.Publisher('probot_set_output_io', SetOutputIO, queue_size=1)

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
        target_pose.pose.orientation.x = 0
        target_pose.pose.orientation.y = 0
        target_pose.pose.orientation.z = 0
        target_pose.pose.orientation.w = 1
        
        # Set pick position 
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        
        traj = self.arm.plan()

        if len(traj.joint_trajectory.points) == 0:
            return False

        self.arm.execute(traj)
        
        return True
    
    def moveSingleJoint(self, index, value):
        group_variable_values = self.arm.get_current_joint_values()
        group_variable_values[index] = value
        self.arm.set_joint_value_target(group_variable_values)
        traj = self.arm.plan()
        self.arm.execute(traj)
    
    def pick(self, x, y):
        if self.moveTo(x, y, 0.004) == True:
            print "Pick Once"
            self.moveSingleJoint(3, 0.0)

            ioOutput = SetOutputIO()
            ioOutput.mask = 1
            ioOutput.status   = SetOutputIO.IO_ON
            self.ioPub.publish(ioOutput)
            
            rospy.sleep(0.05)
            self.moveSingleJoint(3, 0.01)
            
            return True
        else:
            print "Can not pick"
            return False
        
    def place(self, x, y): 
        if self.moveTo(x, y, 0.015) == True:
            print "Place Once"
            
            ioOutput = SetOutputIO()
            ioOutput.mask = 1
            ioOutput.status   = SetOutputIO.IO_OFF
            self.ioPub.publish(ioOutput)

            rospy.sleep(0.15)
        else:
            print "Can not place"

    def shutdown(self):
        # Exit
        print "The demo is shutting down."
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


def demoCtrlCallback(data):
    rospy.loginfo("Demo Receive Ctrl Code[%d]",data.ctrl)
    global demoCtrlCode_
    demoCtrlCode_ = data.ctrl

if __name__ == "__main__":

    time.sleep(5)

    rospy.init_node('probot_sorting_demo')
    rate = rospy.Rate(10)

    rospy.Subscriber("probot_demo_ctrl", DemoCtrl, demoCtrlCallback)

    reg_x = rospy.get_param('/image/reg_x')
    reg_y = rospy.get_param('/image/reg_y')

    # Wait start
    while demoCtrlCode_ is DEMO_STOP:
        time.sleep(0.5)
    
    print "Probot sorting demo start."
    demo = ProbotSortingDemo()
    
    # Initialize position
    demo.moveTo(greenBox[0], greenBox[1], 0.02)

    while not rospy.is_shutdown():
        # Wait start
        while demoCtrlCode_ is DEMO_STOP:
            rate.sleep()

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
            if demo.pick(x_value,  y_value) == True:
                demo.place(redBox[0], redBox[1])
        elif len(response.greenObjList):
            x_value = response.greenObjList[0].position.y * reg_x[0] + reg_x[1]
            y_value = response.greenObjList[0].position.x * reg_y[0] + reg_y[1]
            print "Pick Position: %f, %f"%(x_value, y_value)
            if demo.pick(x_value,  y_value) == True:
                demo.place(greenBox[0], greenBox[1])
        elif len(response.blueObjList):
            x_value = response.blueObjList[0].position.y * reg_x[0] + reg_x[1]
            y_value = response.blueObjList[0].position.x * reg_y[0] + reg_y[1]
            print "Pick Position: %f, %f"%(x_value, y_value)
            if demo.pick(x_value,  y_value) == True:
                demo.place(blueBox[0], blueBox[1])
        elif len(response.blackObjList):
            x_value = response.blackObjList[0].position.y * reg_x[0] + reg_x[1]
            y_value = response.blackObjList[0].position.x * reg_y[0] + reg_y[1]
            print "Pick Position: %f, %f"%(x_value, y_value)
            if demo.pick(x_value,  y_value) == True:
                demo.place(blackBox[0], blackBox[1])

        rate.sleep()

    demo.moveToHome()   
    demo.shutdown()
