#!/usr/bin/env python
import copy
import actionlib
import rospy
import numpy as np
#import pcl

from math import sin, cos
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import Int8, Float32
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import moveit_commander

DES_JOINTS_TOPIC = "/fetch/des_states"

NO_DOF = 7

class FollowTrajectoryClient(object):

    def __init__(self, name, joint_names):
        self.trajectory = JointTrajectory()
        self.sub_des_states = rospy.Subscriber(DES_JOINTS_TOPIC, JointTrajectory, self.handle_des_states, tcp_nodelay=True,
                                           queue_size=1)
        self.client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % name,
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for %s..." % name)
        self.client.wait_for_server()
        self.joint_names = joint_names

    def handle_des_states(self, data):
        self.trajectory = data
        self.trajectory.joint_names = self.joint_names

    def move_to(self):
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = self.trajectory

        self.client.send_goal(follow_goal)
        self.client.wait_for_result()

if __name__ == "__main__":
    # Create a node
    rospy.init_node("fetch_planning_interface")
    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    # Setup clients
    arm_action = FollowTrajectoryClient("arm_controller", ["shoulder_pan_joint", "shoulder_lift_joint", \
                                                             "upperarm_roll_joint", "elbow_flex_joint", \
                                                             "forearm_roll_joint", "wrist_flex_joint", \
                                                             "wrist_roll_joint"])
    rospy.loginfo("Moving the arm with position commands")

    while not rospy.is_shutdown():
        arm_action.move_to()
