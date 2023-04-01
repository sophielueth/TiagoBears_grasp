# TiagoBears_grasp
The grasping pipeline for team TiagoBears.

## Service Usage

Start the Service Node with 
```
roslaunch TiagoBears_grasp grasp_node
```

and query the Services `pick_left`, `pick_right`, `place_left`, `place_right` using TiagoBears_grasp.srv PickPlace.

## Grasp Parameters
The parameters for grasping and parameters specifically for the gripper type used are in `config/*.yaml`.

They can be loaded with 
```
roslaunch TiagoBears_grasp load_config.launch ee:=pal-gripper
```

The parameters will then be loaded onto the param server under the namespace TiagoBears.
`ee:=pal-gripper` could be omitted, as it is the default value. Use actively for `ee:=robotiq-2f-85`.

## Usage

A `Grasp` object, rather a grasping handler, can simply be created via
```
grasp_left=Grasp(is_left=True) # left arm
grasp_right=Grasp(is_left=False) # right arm

grasp_left.pick(cube_center_pose) # Pose from geometry_msgs.msg.Pose
grasp_left.place(place_cube_center_pose)
```

For an example implementation see `TiagoBears_plan/scripts/grasp_node.py`.
