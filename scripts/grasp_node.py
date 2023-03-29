#!/usr/bin/env python

from threading import Lock

import rospy

from TiagoBears_grasp.grasp_class import Grasp
from TiagoBears_grasp.srv import PickPlace, PickPlaceResponse

def in_center(pose):
    if pose.position.y < 0.05 and pose.position.y > -0.05:
        return True
    else:
        return False

if __name__ == '__main__':
    rospy.init_node('TiagoBears_grasp')

    center_part_lock = Lock()

    grasp_left = Grasp(is_left=True)
    grasp_right = Grasp(is_left=False)

    def pick_left(req):
        if in_center(req.target_pose):
            with center_part_lock:
                return PickPlaceResponse(success=grasp_left.pick(req.target_pose))
        else:
            return PickPlaceResponse(success=grasp_left.pick(req.target_pose))
    
    def pick_right(req):
        if in_center(req.target_pose):
            with center_part_lock:
                return PickPlaceResponse(success=grasp_right.pick(req.target_pose))
        else:
            return PickPlaceResponse(success=grasp_right.pick(req.target_pose))
    
    def place_left(req):
        if in_center(req.target_pose):
            with center_part_lock:
                return PickPlaceResponse(success=grasp_left.place(req.target_pose))
        else:
            return PickPlaceResponse(success=grasp_left.place(req.target_pose))
    
    def place_right(req):
        if in_center(req.target_pose):
            with center_part_lock:
                return PickPlaceResponse(success=grasp_right.place(req.target_pose))
        else:
            return PickPlaceResponse(success=grasp_right.place(req.target_pose))
    
    pick_left = rospy.Service('/TiagoBears/pick_left', PickPlace, pick_left)
    pick_right = rospy.Service('/TiagoBears/pick_right', PickPlace, pick_right)
    place_left = rospy.Service('/TiagoBears/place_left', PickPlace, place_left)
    place_right = rospy.Service('/TiagoBears/place_right', PickPlace, place_right)

    print('Ready to grasp!')

    rospy.spin()