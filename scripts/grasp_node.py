#!/usr/bin/env python

from threading import Lock

import rospy

from TiagoBears_grasp.grasp_class import Grasp
from TiagoBears_grasp.srv import PickPlace, PickPlaceResponse, Trigger, TriggerResponse

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
    
    def go_to_start_left(req):
        grasp_left.move_to_start_position()
        return TriggerResponse(res=True)

    def go_to_start_right(req):
        grasp_right.move_to_start_position()
        return TriggerResponse(res=True)

    def go_to_watch_left(req):
        grasp_left.move_to_watch_position()
        return TriggerResponse(res=True)

    def go_to_watch_right(req):
        grasp_right.move_to_watch_position()
        return TriggerResponse(res=True)

    def open_left(req):
        while not grasp_left.set_gripper(grasp_left._gripper_opened): pass
        return TriggerResponse(res=True)
    
    def open_right(req):
        while not grasp_right.set_gripper(grasp_right._gripper_opened): pass
        return TriggerResponse(res=True)
    
    def close_left(req):
        while not grasp_left.set_gripper(grasp_left._gripper_closed): pass
        return TriggerResponse(res=True)
    
    def close_right(req):
        while not grasp_right.set_gripper(grasp_right._gripper_closed): pass
        return TriggerResponse(res=True)

    pick_left = rospy.Service('/TiagoBears/pick_left', PickPlace, pick_left)
    pick_right = rospy.Service('/TiagoBears/pick_right', PickPlace, pick_right)
    place_left = rospy.Service('/TiagoBears/place_left', PickPlace, place_left)
    place_right = rospy.Service('/TiagoBears/place_right', PickPlace, place_right)

    go_to_start_left = rospy.Service('/TiagoBears/go_to_start_left', Trigger, go_to_start_left)
    go_to_start_right = rospy.Service('/TiagoBears/go_to_start_right', Trigger, go_to_start_right)
    go_to_watch_left = rospy.Service('/TiagoBears/go_to_watch_left', Trigger, go_to_watch_left)
    go_to_watch_right = rospy.Service('/TiagoBears/go_to_watch_right', Trigger, go_to_watch_right)

    open_left = rospy.Service('/TiagoBears/open_left_gripper', Trigger, open_left)
    open_right = rospy.Service('/TiagoBears/open_right_gripper', Trigger, open_right)
    close_left = rospy.Service('/TiagoBears/close_left_gripper', Trigger, close_left)
    close_right = rospy.Service('/TiagoBears/close_right_gripper', Trigger, close_right)

    print('=== Ready to grasp! ===')

    rospy.spin()