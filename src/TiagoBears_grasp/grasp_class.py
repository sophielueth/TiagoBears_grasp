#!/usr/bin/env python
from math import pi
import sys
import copy
import numpy as np

import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped
from tf import transformations

from cube_class import Cube

# MAIN FRAME IS base_footprint
# later TODO s: 
# - both arms to move and "work" simulaneously
# - add some params in the init function to a param .yaml and read out from the parameter server
# - add a smart wait pose

# list of (current)Z TODO s:
# - add opening and closing of gripper via gripper_left_controller and gripper_right_controller (Action Client)
# - check endeffector frame of groups, otherwise convert them, add orientation of approach angle 
# - add error handling


class Grasp:
	def __init__(self):
		# later TODO add to param server and read out
		self._approach_angle = pi / 4 # in rad
		self._cube_length = 0.045 # in m
		self._height_over_place = 0.005 # in m 
		
		# self._wait_pose_left = Pose() # later TODO see above
		# self._wait_pose_right = Pose()

		self._look_at_pose_left = Pose() 
		(self._look_at_pose_left.position.x, self._look_at_pose_left.position.y, self._look_at_pose_left.position.z) = (0.626, 0.107, 0.882) # supposed to be sent to gripper_left_grasping_frame
		(self._look_at_pose_left.orientation.x, self._look_at_pose_left.orientation.y, self._look_at_pose_left.orientation.z, self._look_at_pose_left.orientation.w) = (0.774, -0.497, 0.356, -0.165)
		self._look_at_pose_right = Pose()
		(self._look_at_pose_right.position.x, self._look_at_pose_right.position.y, self._look_at_pose_right.position.z) = (0.608, -0.130, 0.882) # supposed to be sent to gripper_right_grasping_frame
		(self._look_at_pose_right.orientation.x, self._look_at_pose_right.orientation.y, self._look_at_pose_right.orientation.z, self._look_at_pose_right.orientation.w) = (0.773, 0.494, 0.364, 0.162)

		# initialize move it for both arms
		moveit_commander.roscpp_initialize(sys.argv)

		## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
		## kinematic model and the robot's current joint states
		self.robot = moveit_commander.RobotCommander(robot_description="robot_description")

		## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
		## for getting, setting, and updating the robot's internal understanding of the
		## surrounding world:
		self.scene = moveit_commander.PlanningSceneInterface()

		## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
		## to a planning group (group of joints).
		## This interface can be used to plan and execute motions:
		group_name_left = "arm_left"  
		group_name_right = "arm_right"
		self.move_group_left = moveit_commander.MoveGroupCommander(group_name_left)
		self.move_group_right = moveit_commander.MoveGroupCommander(group_name_right)

		## Create a `DisplayTrajectory`_ ROS publisher which is used to display
		## trajectories in Rviz:
		# very later TODO: check whether I can have one topic to publish both trajectories
		self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
		                                               moveit_msgs.msg.DisplayTrajectory,
		                                               queue_size=20)

		# reference frame for this robot:
		planning_frame_left = self.move_group_left.get_planning_frame()
		planning_frame_right = self.move_group_right.get_planning_frame()
		print("============ Planning frame left: %s" % planning_frame_left)

		# move to wait position (for now using the watch position as wait position)
		self.move_both_to_watch_position()

		# Stuff that might be useful
		# rospy.init_node('grasp', anonymous=True) is no node
		# eef_link = move_group.get_end_effector_link()
		# group_names = robot.get_group_names()
		# robot.get_current_state()

	def pick(self, cube: Cube):
		# decide on arm to use
		use_left_arm = True if cube.pose.y > 0 else False
		move_group = self.move_group_left if use_left_arm else self.move_group_right

		# create pre-pick (10 cm above pick posistion) & pick position
		pre_pick_poses = self.get_pre_pickplace_poses(cube.pose)
		
		# pick
		plan, _ = move_group.compute_cartesian_path(
			pre_pick_poses,  # waypoints to follow
			0.01,  # eef_step
			0.0)  # jump_threshold
		move_group.execute_plan(plan, wait=True) # later TODO: Somehow enable that the other arm can be started while the first one is in movement
		
		# close gripper
		# TODO
		
		# create post grasp position (same as pre-grasp position) and retract
		plan, _ = move_group.compute_cartesian_path(
			[pre_pick_poses[0]], 0.01, 0.0)  
		move_group.execute_plan(plan, wait=True)

		# move to watch position
		self.move_left_to_watch_position() if use_left_arm else self.move_right_to_watch_position()

		return use_left_arm

	def place(self, place_pose:Pose, use_left_arm:bool):
		# select arm to use
		move_group = self.move_group_left if use_left_arm else self.move_group_right

		# create pre-place (10 cm above place posistion) & place position
		pre_place_poses = self.get_pre_pickplace_poses(place_pose)

		# place
		plan, _ = move_group.compute_cartesian_path(pre_place_poses, 0.01, 0.0)
	
		move_group.execute_plan(plan, wait=True)
		
		# open gripper
		# TODO

		# create post place position (same as pre-place position) and retract
		plan, _ = move_group.compute_cartesian_path(
			[pre_place_poses[0]], 0.01, 0.0)  
		move_group.execute_plan(plan, wait=True)

		# move to watch position
		self.move_left_to_watch_position() if use_left_arm else self.move_right_to_watch_position()

	def get_pre_pickplace_poses(self, cube_pose:Pose):
		target_pose = copy.deepcopy(cube_pose)
		# TODO: use self._approach_angle to calculate it
		R = np.array([[1,  0,  0, 0],
					  [0,  1, -1, 0],
					  [1, -1, -1, 0],
					  [0,  0,  0, np.sqrt(2)]]) / np.sqrt(2)# TODO: adjust orientation to the side of the cube to approach
		target_pose.orientation = transformations.quaternion_from_matrix(R) 
		
		pre_target_pose = copy.deepcopy(target_pose)
		pre_target_pose.z += 0.1 # have the end-effector approach from 10 cm above the cube

		return [pre_target_pose, target_pose]

	def move_left_to_watch_position(self):
		plan, _ = self.move_group_left.compute_cartesian_path([self._look_at_pose_left], 0.01, 0.0)
		self.move_group_left.execute_plan(plan, wait=True)

	def move_right_to_watch_position(self):
		plan, _ = self.move_group_right.compute_cartesian_path([self._look_at_pose_right], 0.01, 0.0)
		self.move_group_right.execute_plan(plan, wait=True)

	def move_both_to_watch_position(self):
		# later TODO: make simultaneuos
		self.move_left_to_watch_position()
		self.move_right_to_watch_position()
