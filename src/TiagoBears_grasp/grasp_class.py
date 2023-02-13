#!/usr/bin/env python
from math import pi
import sys
import copy
import numpy as np

import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from tf import transformations as tr

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
# - add orientation of approach angle 
# - add error handling


class Grasp:
	def __init__(self, is_left, traj_pub):
		# later TODO add to param server and read out
		self._cube_length = 0.045 # in m
		self._height_over_place = 0.005 # in m 

		# self._look_at_pose_left = Pose(Point(0.626, 0.107, 0.882), Quaternion(0.774, -0.497, 0.356, -0.165)) # supposed to be sent to gripper_left_grasping_frame
		# self._look_at_pose_right = Pose(Point(0.608, -0.130, 0.882), Quaternion(0.773, 0.494, 0.364, 0.162)) # supposed to be sent to gripper_right_grasping_frame

		self._arm_straight_pose = [0, 0, 0, 0, 0, 0, 0] # in joint space
		self._start_pose = [0.50, 0.00, 2.00, 1.35, -1.57, 0.70, 0.70] # in joint space
		self._look_at_pose = [0.50, 0.30, 1.25, 1.35, -1.57, 0.70, 0.70]# in joint space

		## Instantiate a `MoveGroupCommander`_ object; it's interface to a planning group (group of joints), used to plan and execute motions:
		group_name = 'arm_left' if is_left else 'arm_right'

		self.move_group = moveit_commander.MoveGroupCommander(group_name)
		self.move_group.set_planning_time(1.0)
		self.move_group.set_num_planning_attempts(5)

		self.display_trajectory_publisher = traj_pub

		# Set end_effector_link frame to gripper_xx_grasping_frame
		ee_link = 'gripper_left_grasping_frame' if is_left else 'gripper_right_grasping_frame'
		self.move_group.set_end_effector_link(ee_link)

		# move to wait position (for now using the watch position as wait position)
		self.move_to_start_position()

		# init action clients to move grippers
		ns = '/gripper_left_controller/follow_joint_trajectory' if is_left else '/gripper_right_controller/follow_joint_trajectory'
		self._gripper_client = SimpleActionClient(ns=ns, ActionSpec=FollowJointTrajectoryAction)
		self._gripper_client.wait_for_server()

		# TODO: check closed values
		self._gripper_closed = [JointTrajectoryPoint(positions=[0.4], time_from_start=rospy.Duration.from_sec(2))]
		self._gripper_opened = [JointTrajectoryPoint(positions=[0.0], time_from_start=rospy.Duration.from_sec(2))]
		self._gripper_joint_names = ['gripper_left_finger_joint'] if is_left else ['gripper_right_finger_joint']

		self.set_gripper(self._gripper_opened)

		self._rotate_0 = np.array([0, 0, 0, 1])
		self._rotate_y_45_deg = np.array([0, 0.382, 0,  0.923])
		self._rotate_z_90_degs = [self._rotate_0, # for the gripper frame to approach from the front
								 np.array([0, 0, 0.707002, -0.7072115]), # left					
								 np.array([0, 0, 1, 0.0002963]), # back
								 np.array([0, 0, 0.707002, 0.7072115])] if is_left else [self._rotate_0, # right for left and front for right 
																						 np.array([0, 0, 0.707002, 0.7072115]), # right
												                                         np.array([0, 0, 1, 0.0002963]), # back
																						 np.array([0, 0, 0.707002, -0.7072115])] # left

	def pick(self, cube):
		while cube.pose == None: pass
		# use_left = True if cube.pose.position.y > 0 else False # should be done somewhere else

		# create pre-pick (10 cm above pick posistion) & pick position
		pick_poses_list = self._get_pre_pick_poses(cube.pose)
		self._execute_pick(pick_poses_list)

		return 0 # should correspond to success

	def place(self, place_pose):
		# select arm to use
		move_group = self.move_group_left if use_left else self.move_group_right

		# create pre-place (10 cm above place posistion) & place position
		place_poses_list = self._get_pre_place_poses(place_pose)
		self._execute_place(place_poses_list)

		return 0 # should be success

	def move_to_start_position(self):
		self.move_group.go(self._arm_straight_pose, wait=True)
		self.move_group.go(self._start_pose, wait=True)
		self.move_group.stop() # to ensure there is not residual movement

	def move_to_watch_position(self):
		self.move_group.go(self._look_at_pose, wait=True)
		self.move_group.stop() # to ensure there is not residual movement

	def set_gripper(self, goal_state):
		# create goal for the action client for the gripper
		goal = FollowJointTrajectoryGoal()

		header = Header()
		header.stamp = rospy.Time.now()
		goal.trajectory = JointTrajectory(header, self._gripper_joint_names, goal_state)
		
		goal.goal_time_tolerance = rospy.Duration.from_sec(0.5)
		
		self._gripper_client.send_goal_and_wait(goal)

	def _execute_pick(self, pick_poses_list):
		# TODO add error handling 
		for pick_poses in pick_poses_list: # if no good path found, remove the -0.45 from the planning
			plan, fraction = self.move_group.compute_cartesian_path(
				pick_poses,  # waypoints to follow
				0.01,  # eef_step
				0.0)  # jump_threshold
			if fraction == 1: # could compute whole trajectory
				used_pose = pick_poses[0]
				break
	
		res = self.move_group.execute(plan, wait=True) # later TODO: Somehow enable that the other arm can be started while the first one is in movement
		
		self.set_gripper(self._gripper_closed)

		self._retract([pick_poses[0]])
		return used_pose

	def _execute_place(self, place_poses_list):
		# TODO: add error handling
		for place_poses in place_poses_list:
			plan, fraction = self.move_group.compute_cartesian_path(place_poses, 0.01, 0.0)
			if fraction == 1: # could compute whole trajectory
				break

		res = self.move_group.execute(plan, wait=True)
		
		self.set_gripper(self._gripper_opened)

		self._retract([place_poses[0]])

	def _retract(self, retract_poses):
		plan, _ = self.move_group.compute_cartesian_path(retract_poses, 0.01, 0.0)  
		self.move_group.execute(plan, wait=True)

		# move to watch position
		self.move_to_watch_position()

	def _get_pre_pick_poses(self, cube_pose):
		poses = []

		for rotate_z in self._rotate_z_90_degs:
			target_pose = copy.deepcopy(cube_pose)
			approach_pose = copy.deepcopy(cube_pose)

			q = quat_to_array(target_pose.orientation)
			# approach from left, right, front or back
			q = tr.quaternion_multiply(q, rotate_z)
			# get hom rot matrix from quaternion
			R = tr.quaternion_matrix(q)
			#  move target pose to -10cm in x direction in its own frame
			(approach_pose.position.x, approach_pose.position.y, approach_pose.position.z) = (approach_pose.position.x, approach_pose.position.y, approach_pose.position.z) - 0.05 * R[:3, 0]

			q = tr.quaternion_multiply(q, self._rotate_y_45_deg) # approach angle to horizontal

			# have the end-effector approach from 10 cm above the cube
			target_pose.orientation = approach_pose.orientation = array_to_quat(q)
			pre_pose = copy.deepcopy(approach_pose)
			pre_pose.position.z += 0.1

			poses.append([pre_pose, approach_pose, target_pose])

		return poses

	def _get_pre_place_poses(self, cube_pose):
		poses = []

		for rotate_z in self._rotate_z_90_degs:	
			target_pose = copy.deepcopy(cube_pose)
			q = quat_to_array(target_pose.orientation)
			q = tr.quaternion_multiply(q, rotate_z) # approach from left, right, front or back

			q = tr.quaternion_multiply(q, self._rotate_y_45_deg) # approach angle to horizontal

			target_pose.orientation = array_to_quat(q)
			pre_pose = copy.deepcopy(target_pose)
			pre_pose.position.z += 0.1 # have the end-effector approach from 10 cm above the cube

			poses.append([pre_pose, target_pose])

		return poses


def array_to_quat(quat_arr):
	return Quaternion(*quat_arr[:])

def quat_to_array(quat):
	return np.array([quat.x, quat.y, quat.z, quat.w])
