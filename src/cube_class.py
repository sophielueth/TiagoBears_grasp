
from geometry_msgs.msg import PoseStamped

import rospy

from color_map_class import Color, ColorMap


class Cube:
	"""
	This class represents one detected cube.
	pos: np.array with shape (3, ): position in relation to the camera frame
	rot: float: angle in rad, between 0 and pi/2
	color: string

	"""
	def __init__(self, pose_topic, top_color: Color = Color.NONE):
		self._stacked = False
		self.pose = None
		self._pose_topic = pose_topic
		self._colormap = ColorMap(top_color=top_color)

		# continuously update the pose:
		rospy.Subscriber(self._pose_topic, PoseStamped, self.update_pose)

	def update_pose(self, pose: PoseStamped):
		self.pose = pose

	# TODO: interface to rotate the ColorMap... Not sure yet whether to connect it automatically to the pose or do it extra