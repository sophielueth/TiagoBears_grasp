#!/usr/bin/env python

import rospy
from TiagoBears_grasp.grasp_class import Grasp
from TiagoBears_grasp.cube_class import Cube

if __name__ == '__main__':
    rospy.init_node('grasp', anonymous=True)

    try:
        grasp = Grasp()
        cube_13 = Cube(13)

        grasp.pick(cube_13)
        print('pick ended')
    except rospy.ROSInterruptException as e:
        print('an exception has occured:')
        print(e)
