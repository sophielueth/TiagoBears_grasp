#!/usr/bin/env python
from math import pi
import sys
import copy

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped

from cube_class import Cube

# MAIN FRAME IS base_footprint
# TODO: add error handling

class Grasp:
	def __init__(self):
		self._approach_angle = pi / 4 # in rad
		self._cube_length = 0.06 # in m
		self._height_over_place = 0.005 # in m 
		
		# self._wait_pose_left = Pose() # TODO add
		# self._wait_pose_right = Pose()

		self._look_at_pose_left = Pose() # later TODO add to param server and read out
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
		group_name_left = "arm_left"  #TODO: check for group names to include the gripper, as we want to put the gripper_right_grasping_frame onto the cube position
		group_name_right = "arm_right"
		self.move_group_left = moveit_commander.MoveGroupCommander(group_name_left)
		self.move_group_right = moveit_commander.MoveGroupCommander(group_name_right)

		## Create a `DisplayTrajectory`_ ROS publisher which is used to display
		## trajectories in Rviz:
		# TODO: check whether I can have one topic to publish both trajectories
		self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
		                                               moveit_msgs.msg.DisplayTrajectory,
		                                               queue_size=20)

		# reference frame for this robot:
		planning_frame_left = self.move_group_left.get_planning_frame()
		planning_frame_right = self.move_group_right.get_planning_frame()
		print("============ Planning frame left: %s" % planning_frame_left)

		# move to wait position (for now using the watch position as wait position)
		move_both_to_watch_position()


		# Stuff that might be useful
		# rospy.init_node('grasp', anonymous=True) is no node
		# eef_link = move_group.get_end_effector_link()
		# group_names = robot.get_group_names()
		# robot.get_current_state()


	def grasp(self, cube: Cube):
		# decide on arm to use
		use_left_arm = True if cube.pose.y > 0 else False
		move_group = self.move_group_left if use_left_arm else self.move_group_right

		# create pre-grasp, grasp position
		pre_grasp_poses = self.get_pre_grasp_poses(cube.pose)
		
		# grasp
		(plan, fraction) = move_group.compute_cartesian_path(
			pre_grasp_poses,  # waypoints to follow
			0.01,  # eef_step
			0.0)  # jump_threshold
		move_group.execute_plan(plan, wait=True) # TODO: Somehow enable that the other arm can be started while the first one is in movement
		# close gripper
		# TODO: find out how to do that
		
		# post grasp - do something?


		# move to watch position
		move_left_to_watch_position() if use_left_arm else move_right_to_watch_position()

		return use_left_arm

	def place(self, place_pose:Pose, use_left_arm:bool)
		# pre-place, place position, post-place
		# place
		# open gripper
		# Also in case of an exception:
		# move to wait position, open gripper


	def get_pre_grasp_poses(self, cube_pose:Pose):
		grasp_pose = copy.deepcopy(cube_pose) # TODO: change the orientation
		pre_grasp_pose = copy.deepcopy(cube_pose)
		pre_grasp_pose.z += 0.1 # have the end-effector approach from 10 cm above the cube

		return [pre_grasp_pose, grasp_pose]

	def get_post_grasp_poses(self, cube_pose:PoseStamped): #TODO: encode which side is used
		post_grasp_pose = cube_pose.pose #TODO: adjust
		return [post_grasp_pose, self._look_at_pose_left]

	def get_pre_place_poses(self, place_pose:PoseStamped):
		pass

	def get_post_place_poses(self, place_pose:PoseStamped):
		pass

	def move_left_to_watch_position(self):
		plan, _ = self.move_group_left.compute_cartesian_path([self._look_at_pose_left], 0.01, 0.0)
		self.move_group_left.execute_plan(plan, wait=True)

	def move_right_to_watch_position(self):
		plan, _ = self.move_group_right.compute_cartesian_path([self._look_at_pose_right], 0.01, 0.0)
		self.move_group_right.execute_plan(plan, wait=True)

	def move_both_to_watch_position(self):
		# later TODO: make simultaneuos
		move_left_to_watch_position()
		move_right_to_watch_position()
