#!/usr/bin/env python
from math import pi
import sys
import copy
import numpy as np

import rospy

from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTolerance
import moveit_msgs.msg

from tf import transformations as tr
import moveit_commander
from actionlib import SimpleActionClient

class Grasp:
	def __init__(self, is_left, ns='/TiagoBears'):
		self.ns = ns
		self._cube_length = rospy.get_param(self.ns + '/cube_length')
		self._height_over_place = rospy.get_param(self.ns + '/height_over_place')
		self._set_z_up = rospy.get_param(self.ns + '/set_z_up')

		self._arm_straight_pose = rospy.get_param(self.ns + '/arm_straight_pose') # joint space
		self._start_pose = rospy.get_param(self.ns + '/start_pose') # joint space
		self._look_at_pose = rospy.get_param(self.ns + '/look_at_pose') # joint space
		self._grasp_offset_x = rospy.get_param(self.ns + '/gripper/grasp_offset_x')
		self._grasp_offset_z = rospy.get_param(self.ns + '/gripper/grasp_offset_z')

		## Instantiate a `MoveGroupCommander`_ object; it's interface to a planning group (group of joints), used to plan and execute motions:
		group_name = 'arm_left' if is_left else 'arm_right'

		self.move_group = moveit_commander.MoveGroupCommander(name=group_name, wait_for_servers=60.0)
		self.move_group.set_planning_time(rospy.get_param(self.ns + '/planning_time'))
		self.move_group.set_num_planning_attempts(rospy.get_param(self.ns + '/num_planning_attempts'))

		# Set end_effector_link frame to gripper_xx_grasping_frame
		ee_link = 'gripper_left_grasping_frame' if is_left else 'gripper_right_grasping_frame'
		self.move_group.set_end_effector_link(ee_link)

		# move to wait position (for now using the watch position as wait position)
		self.move_to_start_position()

		# init action clients to move grippers
		ns = '/gripper_left_controller/follow_joint_trajectory' if is_left else '/gripper_right_controller/follow_joint_trajectory'
		self._gripper_client = SimpleActionClient(ns=ns, ActionSpec=FollowJointTrajectoryAction)
		self._gripper_client.wait_for_server()

		self._gripper_closed = [JointTrajectoryPoint(positions=rospy.get_param(self.ns + '/gripper/gripper_closed'), time_from_start=rospy.Duration.from_sec(6))]
		self._gripper_opened = [JointTrajectoryPoint(positions=rospy.get_param(self.ns + '/gripper/gripper_opened'), time_from_start=rospy.Duration.from_sec(6))]
		self._gripper_joint_names = rospy.get_param(self.ns + '/gripper/joint_names_left') if is_left else rospy.get_param(self.ns + '/gripper/joint_names_right')

		while not self.set_gripper(self._gripper_opened): pass

		self._rotate_0 = np.array(rospy.get_param(self.ns + '/rotate_0'))
		self._approach_ang_hor = np.array(rospy.get_param(self.ns + '/approach_ang_hor'))
		self._grasps = [np.array(grasp) for grasp in rospy.get_param(self.ns + '/grasps')]

		# first pose to move end-effector to in any grasp scenario
		start_x, start_y, start_z = rospy.get_param(self.ns + '/start_grasp_pose_left_pos') if is_left else rospy.get_param(self.ns + '/start_grasp_pose_right_pos')
		start_grasp_pose_quats = rospy.get_param(self.ns + '/start_grasp_pose_left_quat') if is_left else rospy.get_param(self.ns + '/start_grasp_pose_right_quat')
		self._start_grasp_pose = Pose(position=Point(x=start_x, y=start_y, z=start_z), orientation=array_to_quat(start_grasp_pose_quats[0]))
		self._start_grasp_pose_180 = Pose(position=Point(x=start_x, y=start_y, z=start_z), orientation=array_to_quat(start_grasp_pose_quats[1]))
	
		# debug:pick pose publisher
		self._approach_pick_poses_publisher = rospy.Publisher(self.ns + '/approach_pick_poses', PoseArray, queue_size=1)

	def pick(self, cube_pose):
		# if cube_pose.position.z < 0.51: 
		cube_pose.position.z = 0.51 # to avoid scrapping gripper on the table

		if self._set_z_up:
			# set z up
			cube_pose = self._set_z_up(cube_pose)

		# create pre-pick (10 cm above approach posistion) & approach position (1 cube horizonatlly relative to pick) & pick position & post-pick positin (10 cm above pick position)
		pick_poses_list = self._get_pre_pick_poses(cube_pose)
		success = self._execute_pick(cube_pose, pick_poses_list)

		return success

	def place(self, place_pose):
		# create pre-place (10 cm above place posistion) & place position
		place_poses_list = self._get_pre_place_poses(place_pose)
		success = self._execute_place(place_poses_list)

		return success# represents success

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
		goal.trajectory = JointTrajectory(header=header, joint_names=self._gripper_joint_names, points=goal_state)
		
		goal.goal_time_tolerance = rospy.Duration.from_sec(0.5)
		
		self._gripper_client.wait_for_server()
		res = self._gripper_client.send_goal_and_wait(goal, execute_timeout=rospy.Duration.from_sec(30.0), preempt_timeout=rospy.Duration.from_sec(10.0))

		return res == 3 or res == 4

	def _execute_pick(self, target_pose, pick_poses_list):
		remove_approach_pose = False
		ik_solved = False
		
		# sort pick_poses by order of similarity to optimal pose direction: the direct path from the table start pose to the target pose, compare x axis
		optimal_x = np.array([self._start_grasp_pose.position.x - target_pose.position.x, 
							  self._start_grasp_pose.position.y - target_pose.position.y, 
							  self._start_grasp_pose.position.z - target_pose.position.z])
		orient_error = np.sum(np.square([tr.quaternion_matrix(quat_to_array(pick_pose[-2].orientation))[:3, 0] for pick_pose in pick_poses_list] - optimal_x), axis=1)
		indx = np.argsort(orient_error)
		pick_poses_list = pick_poses_list[indx]

		for i in range(2):
			for pick_poses in pick_poses_list: # if no good path found, remove the -0.05 cm x point from the planning (maybe at another one at -0.02, that can then be chosen) 
				if remove_approach_pose:
					pick_poses_approach = [pick_poses[0], pick_poses[-2]]
				else:
					pick_poses_approach = pick_poses[:-1] 
				plan, fraction = self.move_group.compute_cartesian_path(
					pick_poses_approach,  # waypoints to follow, disregard post pick pose
					0.01,  # eef_step
					0.0)  # jump_threshold

				# debug publish pick poses
				pa = PoseArray()
				pa.header.frame_id = 'base_footprint'
				pa.poses = pick_poses
				self._approach_pick_poses_publisher.publish(pa)

				if fraction == 1: # could compute whole trajectory
					ik_solved = True
					break
			if ik_solved:
				break
			elif not remove_approach_pose:
					remove_approach_pose = True
					print 'will remove the approach pose for path planning'
			else:
				print 'no path could be found'
				return False
		
		while not self.set_gripper(self._gripper_opened): pass
		if not self.move_group.execute(plan, wait=True): # success
			print 'motion execution failed'
			self.move_to_watch_position()
			return False

		# we don't use this return value, as it is not a good indicator of success/failure
		while not self.set_gripper(self._gripper_closed): pass

		self._retract([pick_poses[-1]])

		return True

	def _execute_place(self, place_poses_list):
		for place_poses in place_poses_list:
			plan, fraction = self.move_group.compute_cartesian_path(place_poses, 0.01, 0.0)
			if fraction == 1: # could compute whole trajectory
				break
		
		if not fraction == 1:
			print 'no path could be found'
			return False

		if not self.move_group.execute(plan, wait=True):
			print 'motion execution failed'
			return False
		
		# we don't use this return value, as it is not a good indicator of success/failure
		while not self.set_gripper(self._gripper_opened): pass
		
		self._retract([place_poses[0]])

		return True

	def _retract(self, retract_poses):
		plan, _ = self.move_group.compute_cartesian_path(retract_poses, 0.01, 0.0)  
		self.move_group.execute(plan, wait=True)

		# move to watch position
		self.move_to_watch_position()

	def _get_pre_pick_poses(self, cube_pose):
		poses = []

		for index, grasp in enumerate(self._grasps):
			target_pose = copy.deepcopy(cube_pose)
			approach_pose = copy.deepcopy(target_pose)

			q = quat_to_array(target_pose.orientation)
			# approach from left, right, front or back
			q = tr.quaternion_multiply(q, grasp)
			# get hom rot matrix from quaternion
			R = tr.quaternion_matrix(q)
			# hardcode a grasping frame
			(target_pose.position.x, target_pose.position.y, target_pose.position.z) = (target_pose.position.x, target_pose.position.y, target_pose.position.z) - self._grasp_offset_x * R[:3, 0] # in cube's x frame
			(target_pose.position.x, target_pose.position.y, target_pose.position.z) = (target_pose.position.x, target_pose.position.y, target_pose.position.z) + self._grasp_offset_z * R[:3, 2] # in cube's z frame

			#  move target pose to -5cm in x direction in its own frame
			approach_pose = copy.deepcopy(target_pose)
			(approach_pose.position.x, approach_pose.position.y, approach_pose.position.z) = (approach_pose.position.x, approach_pose.position.y, approach_pose.position.z) - 0.05 * R[:3, 0]

			q = tr.quaternion_multiply(q, self._approach_ang_hor) # approach angle to horizontal

			# have the end-effector approach from 10 cm above the cube
			target_pose.orientation = approach_pose.orientation = array_to_quat(q)

			# post grasp pose for retracting, is 10 cm above target pose
			post_pose = copy.deepcopy(target_pose)
			post_pose.position.z += 0.1

			# respect orientation of grasp:
			if index % 2:
				poses.append([self._start_grasp_pose, approach_pose, target_pose, post_pose])
			else:
				poses.append([self._start_grasp_pose_180, approach_pose, target_pose, post_pose])
				


		return np.array(poses)

	def _get_pre_place_poses(self, cube_pose):
		poses = []

		for grasp in self._grasps:	
			target_pose = copy.deepcopy(cube_pose)
			q = quat_to_array(target_pose.orientation)
			q = tr.quaternion_multiply(q, grasp) # approach from left, right, front or back

			R = tr.quaternion_matrix(q)
			# hardcode a grasping frame
			(target_pose.position.x, target_pose.position.y, target_pose.position.z) = (target_pose.position.x, target_pose.position.y, target_pose.position.z) - self._grasp_offset_x * R[:3, 0] # -2 cm in cube's x frame
			(target_pose.position.x, target_pose.position.y, target_pose.position.z) = (target_pose.position.x, target_pose.position.y, target_pose.position.z) + self._grasp_offset_z * R[:3, 2] # +2 cm in cube's z frame

			q = tr.quaternion_multiply(q, self._approach_ang_hor) # approach angle to horizontal

			target_pose.orientation = array_to_quat(q)
			pre_pose = copy.deepcopy(target_pose)
			pre_pose.position.z += 0.1 # have the end-effector approach from 10 cm above the cube

			poses.append([pre_pose, target_pose])

		return poses

	def _set_z_up(self, pose):
		# disregard any orientation other than in the xy-plane aka set z-axis to point perfectly upwards
		q = quat_to_array(pose.orientation)
		x = tr.quaternion_matrix(q)[:3, 0]
		z = np.array([0, 0, 1])
		y = np.cross(z, x)

		R = np.array([x, y, z]).T
		pose.orientation = tr.quaternion_from_matrix(R)

		return pose

def array_to_quat(quat_arr):
	return Quaternion(*quat_arr[:])

def quat_to_array(quat):
	return np.array([quat.x, quat.y, quat.z, quat.w])
