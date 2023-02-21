# TiagoBears_grasp
The grasping pipeline for team TiagoBears

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

grasp_left.pick(cube5) # cube from the TiagoBears_grasp cube class
# might additionally in the future allow for collision checking after gripper closing
grasp_left.pick(cube5, activate_collision_left) # activate_collision_left would be a function
```

For an example implementation see `TiagoBears_plan/scripts/test_node.py`.