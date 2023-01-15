from math import pi
import sys
import copy

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped

from cube_class import Cube

class Grasp:
	def __init__(self):
		self._approach_angle = pi / 4 # in rad
		self._cube_length = 0.06 # in m...?
		self._height_over_place = 0.005 # in m...?
		self._wait_pose_left = PoseStamped() # TODO init
		self._wait_pose_right = PoseStamped()
		self._look_at_pose_left = PoseStamped()
		self._look_at_pose_right = PoseStamped()

		# initialize move it for both arms
		# move to wait position

		moveit_commander.roscpp_initialize(sys.argv)
		# rospy.init_node('grasp', anonymous=True) is no node

		## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
		## kinematic model and the robot's current joint states
		self.robot = moveit_commander.RobotCommander(robot_description="robot_description") # check whether I have to change this default value

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
		# TODO: check whether I can have one topic to publish both trajectories
		self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
		                                               moveit_msgs.msg.DisplayTrajectory,
		                                               queue_size=20)

		# reference frame for this robot:
		planning_frame_left = self.move_group_left.get_planning_frame()
		planning_frame_right = self.move_group_right.get_planning_frame()
		print("============ Planning frame left: %s" % planning_frame_left)

		# Stuff that might be useful
		# eef_link = move_group.get_end_effector_link()
		# group_names = robot.get_group_names()
		# robot.get_current_state()


	def grasp(self, cube: Cube):
		# decide on arm to use
		move_group = None
		# create pre-grasp, grasp position
		pre_grasp_poses = self.get_pre_grasp_poses(cube)
		# grasp
		(plan, fraction) = move_group.compute_cartesian_path(
			pre_grasp_poses,  # waypoints to follow
			0.01,  # eef_step
			0.0)  # jump_threshold
		move_group.execute_plan(plan, wait=True) # TODO: Somehow enable that the other arm can be started while the first one is in movement
		# close gripper
		# post grasp - do something?
		# pre-place, place position, post-place
		# place
		# open gripper
		# Also in case of an exception:
		# move to wait position, open gripper


	def get_pre_grasp_poses(self, cube_pose:PoseStamped):
		waypoints = []
		# depends basically where end_effector_frame exactly lies
		grasp_pose = cube_pose.pose
		pre_grasp_pose = copy.deepcopy(cube_pose)
		# TODO: adjust pre_grasp_pose

		return [pre_grasp_pose, grasp_pose]

	def get_post_grasp_poses(self, cube_pose:PoseStaped): #TODO: encode which side is used
		post_grasp_pose = cube_pose.pose #TODO: adjust
		return [post_grasp_pose, self._look_at_pose_left]

	def get_pre_place_poses(self, place_pose:PoseStamped):
		pass

	def get_post_place_poses(self, place_pose:PoseStamped):
		pass
