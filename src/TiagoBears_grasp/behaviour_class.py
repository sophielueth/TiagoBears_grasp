#!/usr/bin/env python

from TiagoBears_grasp.cube_class import Cube
from geometry_msgs.msg import Pose

# TODO: add this whole thing :D

class Behaviour:
    pass

class Behaviour_stack_per_color(Behaviour):
    def __init__(self) -> None:
        super().__init__()

    def choose_next_cube(self) -> Cube:
        # choose the closest cube to the robot that is lying horizontally on one plane
        pass

    def free_space_needed(self) -> bool:
        pass

    def choose_place_pose(self, cube:Cube) -> Pose:
        # make sure that cube with respect the place pose is 0.5 cm above the plane to place it on
        pass

    def loop(self) -> int:
        # supposed to return an error code, if any - otherwise go on stacking cubes
        pass
