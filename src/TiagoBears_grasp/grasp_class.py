#!/usr/bin/env python
from math import pi
import sys
import copy
import numpy as np

import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose, Point, Quaternion
from tf import transformations

from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTolerance
from actionlib import SimpleActionClient



from cube_class import Cube

# MAIN/Basis FRAME is base_footprint
# later TODO s: 
# - both arms to move and "work" simulaneously
# - add some params in the init function to a param .yaml and read out from the parameter server
# - add a smart wait pose

# list of (current) TODO s:
# - check opening and closing values for the endeffector
# - check endeffector frame of groups, otherwise convert them, add orientation of approach angle 
# - add error handling


class Grasp:
	def __init__(self):
		# later TODO add to param server and read out
		self._approach_angle = pi / 4 # in rad
		self._cube_length = 0.045 # in m
		self._height_over_place = 0.005 # in m 
		
		# self._wait_pose_left = Pose() # later TODO add wait pose above table
		# self._wait_pose_right = Pose()

		self._look_at_pose_left = Pose(Point(0.626, 0.107, 0.882), Quaternion(0.774, -0.497, 0.356, -0.165)) # supposed to be sent to gripper_left_grasping_frame
		self._look_at_pose_right = Pose(Point(0.608, -0.130, 0.882), Quaternion(0.773, 0.494, 0.364, 0.162)) # supposed to be sent to gripper_right_grasping_frame

		# there was a collision when tried the old poses
		# self._look_at_pose_left = Pose(Point(0.4, 0.25, 0.882), Quaternion( 0.9, 0.4, -0.2, 0.4)) # supposed to be sent to gripper_left_grasping_frame
		# self._look_at_pose_right = Pose(Point(0.4, -0.25, 0.882), Quaternion(0.9, 0.4, -0.2, 0.4)) # supposed to be sent to gripper_right_grasping_frame

		## initialize move it for both arms
		moveit_commander.roscpp_initialize(sys.argv)

		## Instantiate a `RobotCommander`_ object. Provides information such as the robot's kinematic model and the robot's current joint states
		self.robot = moveit_commander.RobotCommander(robot_description="robot_description")

		## Instantiate a `PlanningSceneInterface`_ object; it's a remote interface for getting, setting, and updating the robot's internal understanding of the surrounding world:
		self.scene = moveit_commander.PlanningSceneInterface()

		## Instantiate a `MoveGroupCommander`_ object; it's interface to a planning group (group of joints), used to plan and execute motions:
		group_name_left = "arm_left"  
		group_name_right = "arm_right"
		self.move_group_left = moveit_commander.MoveGroupCommander(group_name_left)
		self.move_group_right = moveit_commander.MoveGroupCommander(group_name_right)

		## Create a `DisplayTrajectory`_ ROS publisher which is used to display trajectories in Rviz:
		# very much later TODO: check whether I can have one topic to publish both trajectories
		self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
		                                               moveit_msgs.msg.DisplayTrajectory,
		                                               queue_size=20)

		# reference frame for this robot:
		planning_frame_left = self.move_group_left.get_planning_frame()
		planning_frame_right = self.move_group_right.get_planning_frame()
		print("============ Planning frame left: %s" % planning_frame_left)

		# move to wait position (for now using the watch position as wait position)
		self.move_both_to_watch_position()

		# init action clients to move grippers
		self._gripper_left_client = SimpleActionClient(ns='/gripper_left_controller/follow_joint_trajectory', ActionSpec=FollowJointTrajectoryAction)
		self._gripper_right_client = SimpleActionClient(ns='/gripper_right_controller/follow_joint_trajectory', ActionSpec=FollowJointTrajectoryAction)
		self._gripper_left_client.wait_for_server()
		self._gripper_right_client.wait_for_server()

		# TODO: add correct values
		self._gripper_closed = [JointTrajectoryPoint(positions=[0.022, 0.022], velocities=np.zeros((2,)), time_from_start=rospy.Duration.from_sec(2))]
		self._gripper_opened = [JointTrajectoryPoint(positions=[0.04, 0.04], velocities=np.zeros((2,)), time_from_start=rospy.Duration.from_sec(2))]
		self._gripper_left_joint_names = ['gripper_left_left_finger_joint', 'gripper_left_right_figher_joint']
		self._gripper_right_joint_names = ['gripper_right_left_finger_joint', 'gripper_rightt_right_figher_joint']

		# self.open_gripper()

		# Stuff that might be useful
		# eef_link = move_group.get_end_effector_link()
		# robot.get_current_state()

	# seems ROS doesn't allow determining the argument and the output types
	# def pick(self, cube: Cube) -> bool: 
	def pick(self, cube):
		# decide on arm to use
		use_left = True if cube.pose.y > 0 else False
		move_group = self.move_group_left if use_left else self.move_group_right

		# create pre-pick (10 cm above pick posistion) & pick position
		pick_poses = self._get_pre_pickplace_poses(cube.pose)
		
		self._execute_pick(move_group, pick_poses)
		
		# retract via the pre-pick pose to the watch position
		self._retract(use_left, pick_poses[0])

		return use_left

	def place(self, use_left, place_pose):
		# select arm to use
		move_group = self.move_group_left if use_left else self.move_group_right

		# create pre-place (10 cm above place posistion) & place position
		place_poses = self._get_pre_pickplace_poses(place_pose)

		self._execute_place(use_left, place_poses)

		# create post place position (same as pre-place position) and retract
		self._retract(use_left, place_poses[0])

	def move_left_to_watch_position(self):
		plan, _ = self.move_group_left.compute_cartesian_path([self._look_at_pose_left], 0.01, 0.0)
		self.move_group_left.execute(plan, wait=True)

	def move_right_to_watch_position(self):
		plan, _ = self.move_group_right.compute_cartesian_path([self._look_at_pose_right], 0.01, 0.0)
		self.move_group_right.execute(plan, wait=True)

	def move_both_to_watch_position(self):
		# later TODO: make simultaneuos
		self.move_left_to_watch_position()
		self.move_right_to_watch_position()

	def close_gripper(self, use_left):
		gripper_client, joint_names = self._gripper_left_client, self._gripper_left_joint_names if use_left else self._gripper_right_client, self._gripper_right_joint_names
		
		# create goal for the action client for the gripper
		goal = FollowJointTrajectoryGoal()

		header = Header()
		header.stamp = rospy.Time.now()
		goal.trajectory = JointTrajectory(header, joint_names, self._gripper_closed)
		
		goal.path_tolerance = JointTolerance() # choose the default values
		goal.goal_tolerance = goal.path_tolerance
		goal.goal_time_tolerance = rospy.Duration.from_sec(1.0)
		
		gripper_client.send_goal_and_wait(goal=goal, Duration=rospy.Duration.from_sec(5.0))

	def open_gripper(self, use_left=True):
		print(self._gripper_left_client, self._gripper_left_joint_names)
		gripper_client, joint_names = self._gripper_left_client, self._gripper_left_joint_names if use_left else self._gripper_right_client, self._gripper_right_joint_names
		
		# create goal for the action client for the gripper
		goal = FollowJointTrajectoryGoal()

		header = Header()
		header.stamp = rospy.Time.now()
		goal.trajectory = JointTrajectory(header, joint_names, self._gripper_opened)
		
		goal.path_tolerance = JointTolerance() # choose the default values
		goal.goal_tolerance = goal.path_tolerance
		goal.goal_time_tolerance = rospy.Duration.from_sec(1.0)
		
		gripper_client.send_goal_and_wait(goal=goal, Duration=rospy.Duration.from_sec(5.0))

	def _execute_pick(self, use_left, pick_poses):
		move_group = self.move_group_left if use_left else self.move_group_right

		plan, _ = move_group.compute_cartesian_path(
			pick_poses,  # waypoints to follow
			0.01,  # eef_step
			0.0)  # jump_threshold
		move_group.execute_plan(plan, wait=True) # later TODO: Somehow enable that the other arm can be started while the first one is in movement
		
		self.close_gripper()

	def _execute_place(self, use_left, place_poses):
		move_group = self.move_group_left if use_left else self.move_group_right

		plan, _ = move_group.compute_cartesian_path(place_poses, 0.01, 0.0)
	
		move_group.execute_plan(plan, wait=True)
		
		self.open_gripper()

	def _retract(self, use_left, retract_pose):
		move_group = self.move_group_left if use_left else self.move_group_right

		plan, _ = move_group.compute_cartesian_path(
			[retract_pose], 0.01, 0.0)  
		move_group.execute_plan(plan, wait=True)

		# move to watch position
		self.move_left_to_watch_position() if use_left else self.move_right_to_watch_position()

	def _get_pre_pickplace_poses(self, cube_pose):
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