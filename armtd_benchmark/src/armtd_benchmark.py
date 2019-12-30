#!/usr/bin/env python
import sys
import os
import csv
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

#directory path: ~/ws_moveit/src/armtd_benchmark/test_worlds/world_1/obstacles_x_world_1.csv
#output directory path: = ~/ws_moveit/src/armtd_benchmark/chomp_stats/chomp_ouput_0x_0y.txt
def all_close(goal, actual, tolerance):
	"""
	Convenience method for checking if a list of values are within tolerance
	goal = A list of floats, a Pose, or PoseStamped
	actual = A list of floats, a Pose, or PoseStamped
	tolerance = A float
	returns a bool 

	"""

	all_equal = True
	if type(goal) is list:
		for index in range(len(goal)):
			if abs(actual[index] - goal[index]) > tolerance:
				return False
	elif type(goal) is geometry_msgs.msg.PoseStamped:
		return all_close(goal.pose, actual.pose, tolerance)

	elif type(goal) is geometry_msgs.msg.Pose:
		return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

	return True

class PythonBenchmarkInterface(object):
	def __init__(self):

		#------------ Instantiate interfaces with various aspects of Robot and Scene --------------

		# to support cooperative multiple inheritance 
		super(PythonBenchmarkInterface, self).__init__()

		# Initialize moveit_commander and a rospy node
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('armtd_benchmark', anonymous=True)

		# Instantiate a RobotCommander object. Provides the robot's kinematic model and joint states
		self.robot = moveit_commander.RobotCommander()

		# Instantiate a planning scene interface
		self.scene = moveit_commander.PlanningSceneInterface()

		# Instantiate a MoveGroupCommander to interface with a planning group (group of joints)
		group_name = "arm"
		#group_name = "panda_arm"
		self.move_group = moveit_commander.MoveGroupCommander(group_name)

		# Create a DisplayTrajectory ROS publisher to display shit in RViz
		self.display_trajectory_publisher = rospy.Publisher('move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

	def go_to_joint_state(self, joint_goal):
		self.move_group.set_joint_value_target(joint_goal)
		self.move_group.go(joint_goal, wait=True)
		self.move_group.stop()

	def plan_joint_goal(self, joint_goal):
		self.move_group.set_joint_value_target(joint_goal)
		return self.move_group.plan()

	def execute_joint_goal_plan(self, plan):
		self.move_group.execute(plan, wait=True)

	# this function is called to assure enough time for box update to be published
	def wait_for_box_update(self, box_name, timeout=4):

		start = rospy.get_time()
		seconds = rospy.get_time()

		while(seconds - start < timeout) and not rospy.is_shutdown():

			#check if box has been successfully added to scene
			added = box_name in self.scene.get_known_object_names()

			if added:
				#print box_name
				return True

			rospy.sleep(0.1)
			seconds = rospy.get_time()

		return False


	def add_box(self, box_name, box_pose, box_size, timeout=4):

		self.scene.add_box(box_name, box_pose, box_size)
		#print box_name
		return self.wait_for_box_update(box_name, timeout)
		#return self.wait_for_box_update(box_name, 10)

	def display_trajectory(self, plan, start):
		#self.display_trajectory_publisher
		display_trajectory = moveit_msgs.msg.DisplayTrajectory()
		display_trajectory.trajectory_start = start
		display_trajectory.trajectory.append(plan)
		self.display_trajectory_publisher.publish(display_trajectory)
		
	def add_scene_obstacle(self, obstacle, obstacle_num):

		obstacle_name = "obstacle" + str(obstacle_num)

		obstacle_pose = geometry_msgs.msg.PoseStamped()
		obstacle_pose.header.frame_id = self.move_group.get_planning_frame()
		#print box_pose.header.frame_id
		obstacle_pose.pose.orientation.w = 1.0
		obstacle_pose.pose.position.x = obstacle[0]
		obstacle_pose.pose.position.y = obstacle[1]
		obstacle_pose.pose.position.z = obstacle[2]

		obstacle_size = (obstacle[3], obstacle[4], obstacle[5])
		self.add_box(obstacle_name, obstacle_pose, obstacle_size)
		return 

	def add_all_obstacles(self, obstacles):

		obstacle_tag = 0

		for obstacle in obstacles:
			self.add_scene_obstacle(obstacle, obstacle_tag)
			obstacle_tag += 1

	def set_tolerance_for_goal(self, tol):
		self.move_group.set_goal_joint_tolerance(tol)
		return

def list_string_to_float(my_list):
	float_list = []
	
	for item in my_list:

		row = [float(i) for i in item]
		float_list.append(row)
	return float_list



def main():
	try:

		# Read filename as system arg if you are testing 
		# this script independently
		
		#filename = sys.argv[1]
		filename = os.environ['FILE_IN']
		test_scene = open(filename, 'r')

		reader = csv.reader(test_scene)

		data = list(reader)

		start_position = [float(i) for i in data[0]]
		goal_position = [float(i) for i in data[1]]

		obstacles = list_string_to_float(data[3:])

		my_group = PythonBenchmarkInterface()

		# To set tolerance, uncomment this line with desired value
		# my_group.set_tolerance_for_goal(0.2)

		default_wrist_roll = 0.0
		start_position.append(default_wrist_roll)
		goal_position.append(default_wrist_roll)
		#my_group.go_to_joint_state(start_position)
		#my_group.go_to_joint_state(goal_position)
		my_group.go_to_joint_state(start_position)

		#print "Press Enter to add obstacles"
		#raw_input()

		my_group.add_all_obstacles(obstacles)

		first_obstacle = obstacles[0]

		my_group.add_scene_obstacle(first_obstacle, 0)

		rospy.sleep(1.5)
		#print "Press enter to continue"
		#raw_input()
		my_group.go_to_joint_state(goal_position)

	except rospy.ROSInterruptException:
		return


	
if __name__ == '__main__':
	main()