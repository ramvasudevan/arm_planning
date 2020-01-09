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

FREQ = 10
NO_DOF = 7
POS_CTRL = 0 # 1 - position commands
             # 0 - velocity commands

class FollowTrajectoryClient(object):

    def __init__(self, name, joint_names):
        self.positions = np.zeros(NO_DOF)
        self.position_prev = np.zeros(NO_DOF)
        self.dt = 1.0/FREQ
        self.velocties = np.zeros(NO_DOF)
        self.pos_ctrl = POS_CTRL
        self.sub_des_states = rospy.Subscriber(DES_JOINTS_TOPIC, JointState, self.handle_des_states, tcp_nodelay=True,
                                           queue_size=1)
        self.client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % name,
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for %s..." % name)
        self.client.wait_for_server()
        self.joint_names = joint_names

    def handle_des_states(self, data):
        if (len(data.position) == NO_DOF):
            self.pos_ctrl = 1
            for i in range(0, len(data.name)):
                if (data.name[i] == "shoulder_pan_joint"):
                    self.positions[0] = data.position[i]
                if (data.name[i] == "shoulder_lift_joint"):
                    self.positions[1] = data.position[i]
                if (data.name[i] == "upperarm_roll_joint"):
                    self.positions[2] = data.position[i]
                if (data.name[i] == "elbow_flex_joint"):
                    self.positions[3] = data.position[i]
                if (data.name[i] == "forearm_roll_joint"):
                    self.positions[4] = data.position[i]
                if (data.name[i] == "wrist_flex_joint"):
                    self.positions[5] = data.position[i]
                if (data.name[i] == "wrist_roll_joint"):
                    self.positions[6] = data.position[i]
        if (len(data.velocity) == NO_DOF):
            self.pos_ctrl = 0
            for i in range(0, len(data.name)):
                if (data.name[i] == "shoulder_pan_joint"):
                    self.velocties[0] = data.velocity[i]
                if (data.name[i] == "shoulder_lift_joint"):
                    self.velocties[1] = data.velocity[i]
                if (data.name[i] == "upperarm_roll_joint"):
                    self.velocties[2] = data.velocity[i]
                if (data.name[i] == "elbow_flex_joint"):
                    self.velocties[3] = data.velocity[i]
                if (data.name[i] == "forearm_roll_joint"):
                    self.velocties[4] = data.velocity[i]
                if (data.name[i] == "wrist_flex_joint"):
                    self.velocties[5] = data.velocity[i]
                if (data.name[i] == "wrist_roll_joint"):
                    self.velocties[6] = data.velocity[i]

    def move_to_positions(self, positions, duration=5.0):
        if len(self.joint_names) != len(positions):
            print("Invalid trajectory position")
            return False
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = positions
        trajectory.points[0].velocities = [0.0 for _ in positions]
        trajectory.points[0].accelerations = [0.0 for _ in positions]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        self.client.send_goal(follow_goal)
        self.client.wait_for_result()

    def move_to_velocity(self, positions, velocity, duration=5.0):
        if len(self.joint_names) != len(velocity):
            print("Invalid trajectory position")
            return False
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = positions
        trajectory.points[0].velocities = velocity
        trajectory.points[0].accelerations = [0.0 for _ in velocity]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        self.client.send_goal(follow_goal)
        self.client.wait_for_result()

    def move_to_without_wait(self, positions, duration=5.0):
        if len(self.joint_names) != len(positions):
            print("Invalid trajectory position")
            return False
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = positions
        trajectory.points[0].velocities = [0.0 for _ in positions]
        trajectory.points[0].accelerations = [0.0 for _ in positions]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        self.client.send_goal(follow_goal)
        #self.client.wait_for_result()


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
        if (arm_action.pos_ctrl == 1):
            arm_action.move_to_positions(arm_action.positions)
        elif (arm_action.pos_ctrl == 0):
            arm_action.move_to_velocity(arm_action.positions, arm_action.velocties)
