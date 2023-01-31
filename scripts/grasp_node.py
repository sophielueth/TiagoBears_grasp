#!/usr/bin/env python

import sys
import rospy
from TiagoBears_grasp.grasp_class import Grasp
from TiagoBears_grasp.cube_class import Cube

from geometry_msgs.msg import Pose, Point, Quaternion

if __name__ == '__main__':
    try:
        rospy.init_node('grasp')

        grasp = Grasp()
        cubes = []

        for i in range(28):
            cubes.append(Cube(i))

        place_pose_left = Pose(position=Point(x=0.765, y=0.335, z=0.505), orientation=Quaternion(w=1.0))
        place_pose_right = Pose(position=Point(x=0.765, y=-0.335, z=0.505), orientation=Quaternion(w=1.0))

        while len(cubes) > 23:
            # choose closest cube
            min_dist_sq = 100 #m, should be impossible
            min_ind = -1
            for index, cube in enumerate(cubes):
                while cube.pose == None: pass
                # dist_sq = cube.pose.position.x**2 + cube.pose.position.y**2 + (cube.pose.position.z-1.0)**2
                dist_sq = (cube.pose.position.x-0.2)**2 + (abs(cube.pose.position.y) - 0.375)**2
                if dist_sq < min_dist_sq:
                    min_dist_sq = dist_sq
                    min_ind = index

            print ('=== Trying to pick cube %d ===', min_ind)
            cube = cubes.pop(min_ind)
            try:
                use_left = grasp.pick(cube)

                place_pose = place_pose_left if use_left else place_pose_right
                print ('=== Trying to pick cube %d ===', min_ind)

                grasp.place(use_left, place_pose)

                # update pose
                if use_left:
                    if place_pose.position.y > 0.06:
                        place_pose.position.y -= 0.06 # move 6 cm to the right, check for next line
                    else:
                        place_pose.position.x -= 0.06 # start next line
                        place_pose.position.y = 0.335
                else:
                    if place_pose.position.y < -0.06:
                        place_pose.position.y += 0.06 # move 6 cm to the right, check for next line
                    else:
                        place_pose.position.x -= 0.06 # start next line
                        place_pose.position.y = -0.335

            except rospy.ROSInterruptException as e:
                print('an exception has occured:')
                print(e)
        except KeyboardInterrupt:
            sys.exit()
