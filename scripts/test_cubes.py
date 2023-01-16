#!/usr/bin/env python
import rospy
from TiagoBears_grasp.cube_class import Cube
import time

# initialize a node
node_name='cube'
rospy.init_node(node_name, anonymous=True)

for i in range(28):
    cube=Cube(id=i)
    time.sleep(1)
    print("cube",i,cube.get_color())