#!/usr/bin/env python
import rospy

from std_msgs.msg import String # for color
from nav_msgs.msg import Odometry # for pose
from geometry_msgs.msg import Quaternion

from collections import deque
from enum import Enum

import tf.transformations as tr
import numpy as np
np.set_printoptions(precision=2, suppress=True)

class Color(Enum):
	# 0,     1,    2,      3,      4,      5,
	RED, GREEN, BLUE, YELLOW, SILVER, NUMBER = range(6)


class ColorMap:
	""" For proper color mapping we should detect the color
		with the letter (or the drawing) in a correct form
		if the letter is upside down, then this complicates the problem
		plus : we have to keep in mind that we have some symmetric letters (O,Z,..)
		Possible solution:
			always rotate left and then build the rotation matrices
	"""

	def __init__(self, color_topic):
		self._color_topic = color_topic
		self.left_rotation = None
		self.up_rotation = None
		self.build_rotation_matrices()

	def build_rotation_matrices(self):
		# Potential cycles: Red, Green, Blue, Yellow - [0,1,2,3]
		# 					Red, Silver, Blue, Number - [0,4,2,5]
		# 					Green, Silver, Yellow, Number - [1,4,3,5]
		top_color = rospy.wait_for_message(self._color_topic, String)
		top = Color[top_color].value
		# rotate the cube in hand
		left_color = rospy.wait_for_message(self._color_topic, String)
		left = Color[left_color].value
		# left now is top
		if top < 4:
			if left < 4:
				# We are in the first cycle
				self.left_rotation = deque([left, (top+2) % 4, (left+2) % 4, top])
				# the up rotation depend on whether the letter is normal or upside down
				# the letter is normal if the left rotation is monotonic (except from 3 to 0) [0,1,2,3] and its rotations
				# the letter is upside down if the left rotation matrix is reversed (except from 0 to 3) [3,2,1,0] and its rotations
				if (left == 0 and top == 3) or (left < 3 and top < left):
					self.up_rotation = deque([left, 4, (left+2) % 4, 5])
				else:
					self.up_rotation = deque([left, 5, (left+2) % 4, 4])
			else:
				# if the left rotation leads to a number or silver
				# we can recognise the opposite surface
				second = 5 if left == 4 else 4
				# given we know the top surface already
				# we can form the rotation matrices
				self.left_rotation = deque([left, (top+2) % 4, second, top])
				self.up_rotation = deque([left, (top+1+2) % 4, second, (top+1) % 4])
		else:
			# if the top surface is a number of a silver
			# we can recognise the opposite surface
			second = 5 if top == 4 else 4
			# given we know the left surface already
			# we can form the rotation matrices
			self.left_rotation = deque([left, second, (left+2) % 4, top])
			self.up_rotation = deque([(left+1)%4, second, (left+1+2) % 4, top])

	def rotate_left(self):
		self.left_rotation.rotate(-1)
		self.up_rotation = deque([self.left_rotation[0], self.up_rotation[1], self.left_rotation[2],self.up_rotation[3]])

	def rotate_right(self):
		self.left_rotation.rotate(1)
		self.up_rotation = deque([self.left_rotation[0], self.up_rotation[1], self.left_rotation[2],self.up_rotation[3]])

	def rotate_up(self):
		self.up_rotation.rotate(-1)
		self.left_rotation = deque([self.up_rotation[0], self.left_rotation[1], self.up_rotation[2],self.left_rotation[3]])

	def rotate_down(self):
		self.up_rotation.rotate(1)
		self.left_rotation = deque([self.up_rotation[0], self.left_rotation[1], self.up_rotation[2],self.left_rotation[3]])

	def get_rotation_to_color(self, target_color):
		rotate_left = 0
		rotate_up = 0
		for i in range(1, 4):
			if self.left_rotation[i] == target_color:
				rotate_left = i
			if self.up_rotation[i] == target_color:
				rotate_up = i
		return {'rotate_left': rotate_left, 'rotate_up': rotate_up}


class Cube:
	"""
	This class represents one detected cube.
	id: an ID number bertween [1,28]
	pos: np.array with shape (3, ): position in relation to the camera frame
	rot: float: angle in rad, between 0 and pi/2
	color: string

	"""
	def __init__(self, id):
		self.id = id
		self.stacked = False

		self.pose = None
		self._pose_topic = '/cube_{0}_odom'.format(id)
		# continuously update the pose:
		# subscribe to the pose estimation topic
		self.pose_sub=rospy.Subscriber(self._pose_topic, Odometry, self.update_pose, queue_size=1)
		self._pose_topic_updated = '/cube_{0}_odom_updated'.format(id)
		self.pose_pub=rospy.Publisher(self._pose_topic_updated, Odometry, queue_size=1)
		
		self._color_topic='/cube_{0}_color'.format(id)
		# get the top color from the color detection node
		# self.color= rospy.wait_for_message(self._color_topic, String)
		# create the color map
		# self._colormap = ColorMap(self.color_topic)
		self.counter=0

	def correct_cube_rotation_matrix(self, R):
		for i in range(3,0,-1):
			ind=np.unravel_index(np.argmax(abs(R[:i,:i]), axis=None), (i,i))
			# the maximum should be a positive number
			if R[ind[0],ind[1]]<0:
				R[ind[0]]*=-1
			# swap colums
			R[[ind[0],i-1]]=R[[i-1,ind[0]]]
			# swap rows
			R[:, [ind[1],i-1]]=R[:, [i-1,ind[1]]]
		return R

	def update_pose(self, msg):
		""" A callback function to update the pose whenever the pose subscriber recieves a topic
		"""
		q = msg.pose.pose.orientation
		if self.counter<1 and self.id==12:
			R=tr.quaternion_matrix([q.x, q.y, q.z, q.w])
			new_R=self.correct_cube_rotation_matrix(R)
			new_q=Quaternion(*tr.quaternion_from_matrix(new_R)[:])
			msg.pose.pose.orientation=new_q
			self.pose_pub.publish(msg)
		self.counter+=1

	def update_color(self):
		""" A function to update the colormap using the color detection node when needed
		"""
		# update the color map
		self._colormap.build_rotation_matrices()