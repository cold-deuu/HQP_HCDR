#! /usr/bin/env python3

    
      
from __future__ import print_function
from six.moves import input
import sys
import copy
import rospy
import geometry_msgs.msg
import actionlib
import cdpr_controllers.msg
import cmd, os
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import tf
from tf.transformations import quaternion_matrix
import math
from std_msgs.msg import Float64
from copy import deepcopy 
from gazebo_msgs.msg import LinkState
#import eigenpy
#eigenpy.switchToNumpyArray()


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class ControlSuiteShell(cmd.Cmd):
    intro = bcolors.OKBLUE + "Welcome to the control suite shell.\nType help or ? to list commands.\n" + bcolors.ENDC
    prompt = "(csuite) "
    def __init__(self):
          cmd.Cmd.__init__(self)
          rospy.init_node('cdpr_actions_client')
        #   self.platform_ctrl_client = actionlib.SimpleActionClient('platform_control', trajectory_generator.msg.platformAction)
        #   self.platform_ctrl_client.wait_for_server()
        #   self.joint_ctrl_client = actionlib.SimpleActionClient('joint_posture_control', trajectory_generator.msg.JointPostureAction)
        #   self.joint_ctrl_client.wait_for_server()
          self.se3_ctrl_client = actionlib.SimpleActionClient('cdpr_controllers/wholebody_control', cdpr_controllers.msg.SE3Action)
          self.se3_ctrl_client.wait_for_server()
          self.arm_joint_client = actionlib.SimpleActionClient('cdpr_controllers/arm_joint_control', cdpr_controllers.msg.JointPostureAction)
          self.arm_joint_client.wait_for_server()

    def do_home(self, arg):
        print("JointPosture Control")
        goal = cdpr_controllers.msg.JointPostureGoal
        goal.duration = 2.0
        goal.target_joints = JointState()
        goal.target_joints.position = np.array([0.0, -0.785398163, 0.0, -2.35619449, 0, 1.57, 0.785398163397])

        self.arm_joint_client.send_goal(goal)
        self.arm_joint_client.wait_for_result()
        if (self.arm_joint_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")



    def do_wholebody(self, arg):
        print ("Wholebody Control")
        goal = cdpr_controllers.msg.SE3Goal
        goal.duration = 5
        goal.target_pose = Pose()
        
        goal.target_pose.position.x = -0.1
        goal.target_pose.position.y = 0.1
        goal.target_pose.position.z = -0.1

        goal.target_pose.orientation.x = 0.258819
        goal.target_pose.orientation.y = 0
        goal.target_pose.orientation.z = 0
        goal.target_pose.orientation.w = 0.9659258

        goal.relative = True
        goal.wholebody = True

        self.se3_ctrl_client.send_goal(goal)
        self.se3_ctrl_client.wait_for_result()
        if (self.se3_ctrl_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")


    def do_wholebody2(self, arg):
        print ("Wholebody Control")
        goal = cdpr_controllers.msg.SE3Goal
        goal.duration = 10
        goal.target_pose = Pose()
        
        goal.target_pose.position.x = 1.0
        goal.target_pose.position.y = 0.2
        goal.target_pose.position.z = 0.2

        goal.target_pose.orientation.x = 0
        goal.target_pose.orientation.y = 0
        goal.target_pose.orientation.z = 0
        goal.target_pose.orientation.w = 1

        goal.relative = True
        goal.wholebody = True

        self.se3_ctrl_client.send_goal(goal)
        self.se3_ctrl_client.wait_for_result()
        if (self.se3_ctrl_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")

    
    def do_quit(self, arg):
          'do quit'
          return True

if __name__ == '__main__':
    ControlSuiteShell().cmdloop()
