from enum import Enum


class Color(Enum):
	RED, GREEN, BLUE, NONE = range(3)


class ColorMap:
	# TODO:Maybe add characters later on... Then it might be more useful to have a 'plane' class that combines letter and color
	# TODO: Maybe a function would be useful, that gives back the shortest list of rotations to get a certain top color

	""" top is supposed to be the plane whose normal is pointing most to the ceiling
	    front is the plane with the normal pointing most the the robot's side of the table
	    the rotations are defined from a view onto the front plane.
	"""

	def __init__(self,
	             top_color: Color = Color.NONE,
	             bottom_color: Color = Color.NONE,
	             front_color: Color = Color.NONE,
	             back_color: Color = Color.NONE,
	             left_color: Color = Color.NONE,
	             right_color: Color = Color.NONE):
		self.top_color = top_color
		self.bottom_color = bottom_color
		self.front_color = front_color
		self.back_color = back_color
		self.left_color = left_color
		self.right_color = right_color

	def rotate_up(self):
		old_top_color = self.top_color
		self.top_color = self.front_color
		self.front_color = self.bottom_color
		self.bottom_color = self.back_color
		self.back_color = old_top_color

	def rotate_down(self):
		old_top_color = self.top_color
		self.top_color = self.back_color
		self.back_color = self.bottom_color
		self.bottom_color = self.front_color
		self.front_colot = old_top_color

	def rotate_clockwise(self):
		old_top_color = self.top_color
		self.top_color = self.left_color
		self.left_color = self.bottom_color
		self.bottom_color = self.right_color
		self.right_color = old_top_color

	def rotate_counterclock(self):
		old_top_color = self.top_color
		self.top_color = self.right_colorr
		self.right_color = self.bottom_color
		self.bottom_color = self.left_color
		self.left_color = old_top_color

	def rotate_right(self):
		old_front_color = self.front_color
		self.front_color = self.left_color
		self.left_color = self.back_color
		self.back_color = self.right_color
		self.right_color = old_front_color

	def rotate_left(self):
		old_front_color = self.front_color
		self.front_color = self.right_color
		self.right_color = self.back_color
		self.back_color = self.self.left_color
		self.left_color = self.old_front_color
