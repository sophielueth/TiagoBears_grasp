#!/usr/bin/env python

import rospy
from TiagoBears_grasp.grasp_class import Grasp
from TiagoBears_grasp.cube_class import Cube

if __name == '__main__':
    try:
        grasp = Grasp()
        cube_13 = Cube(13)

        grasp.pick(cube_13)
        print('pick ended')
    except rospy.ROSInterruptException:
        print('an exception has occured')

