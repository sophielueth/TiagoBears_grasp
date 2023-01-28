#!/usr/bin/env python
import rospy
from TiagoBears_grasp.cube_class import Cube
import time

# initialize a node
node_name='cube'
rospy.init_node(node_name, anonymous=True)

cubes=[]
for i in range(28):
    cubes.append(Cube(id=i))
rospy.spin()