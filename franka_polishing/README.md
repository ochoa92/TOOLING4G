## Launcher:
roslaunch  package_name launch_file_name.launch

## To run a node:
rosrun package_name node_name

## Compile all packages:
catkin_make

## Compile a specific package
catkin_make --pkg package_name

# Example:
roslaunch franka_polishing cartesian_impedance_controller.launch

# To recover from errors and reflexes the franka_control::ErrorRecoveryAction can be called. That can be done from an action client or by simply publishing on the action goal topic:
rostopic pub -1 /franka_control/error_recovery/goal franka_control/ErrorRecoveryActionGoal "{}"
