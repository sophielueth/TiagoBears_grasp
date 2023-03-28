#!/usr/bin/python

from threading import Lock

import rospy

from TiagoBears_grasp.grasp_class import Grasp
from TiagoBears_grasp.srv import PickPlace, PickPlaceResponse

def in_center(pose):
    if pose.position.x < 0.15 and pose.position.x > - 0.15:
        return True
    else:
        return False

if __name__ == '__main__':
    rospy.init_node('TiagoBears_grasp')

    center_part_lock = Lock()

    grasp_left = Grasp(is_left=True)
    grasp_right = Grasp(is_left=False)

    def pick_left(req):
        if in_center(req.pose):
            with center_part_lock:
                return PickPlaceResponse(grasp_left.pick(req.pose)).data
        else:
            return PickPlaceResponse(grasp_left.pick(req.pose)).data
    
    def pick_right(req):
        if in_center(req.pose):
            with center_part_lock:
                return PickPlaceResponse(grasp_right.pick(req.pose)).data
        else:
            return PickPlaceResponse(grasp_right.pick(req.pose)).data
    
    def place_left(req):
        if in_center(req.pose):
            with center_part_lock:
                return PickPlaceResponse(grasp_left.place(req.pose)).data
        else:
            return PickPlaceResponse(grasp_left.place(req.pose)).data
    
    def place_right(req):
        if in_center(req.pose):
            with center_part_lock:
                return PickPlaceResponse(grasp_right.place(req.pose)).data
        else:
            return PickPlaceResponse(grasp_right.place(req.pose)).data
    
    pick_left = rospy.Service('pick_left', PickPlace, pick_left)
    pick_right = rospy.Service('pick_right', PickPlace, pick_right)
    place_left = rospy.Service('place_left', PickPlace, place_left)
    place_right = rospy.Service('place_right', PickPlace, place_right)

    print('Ready to grasp!')

    rospy.spin()