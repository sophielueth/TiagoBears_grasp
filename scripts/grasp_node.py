#!/usr/bin/env python

import rospy
from TiagoBears_grasp.grasp_class import Grasp
from TiagoBears_grasp.cube_class import Cube

if __name == '__main__':
    try:
        Grasp()
        Cube(10)
    except rospy.ROSInterruptException:
        print('an exception has occured')

        