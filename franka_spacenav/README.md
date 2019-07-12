## Launcher:
roslaunch  package_name launch_file_name.launch

## To run a node:
rosrun package_name node_name

## Compile all packages:
catkin_make

## Compile a specific package
catkin_make --pkg package_name

# Example:
roslaunch franka_spacenav cartesian_impedance_controller.launch
