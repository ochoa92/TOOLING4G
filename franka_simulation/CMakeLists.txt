cmake_minimum_required(VERSION 2.8.3)
project(franka_simulation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14 -Wall -O3)


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  controller_manager_msgs
  controller_interface
  effort_controllers
  gazebo_ros
  joint_state_controller
  joint_state_publisher
  joint_trajectory_controller
  robot_state_publisher
  roscpp
  std_msgs
  tf
  xacro
  sensor_msgs
  eigen_conversions
  kdl_parser
  gazebo_msgs
  urdf
  geometry_msgs
  spacenav_node
  dynamic_reconfigure
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(gazebo REQUIRED)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/compliance_param.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
   LIBRARIES ${PROJECT_NAME}

   CATKIN_DEPENDS
   roscpp
   std_msgs
   sensor_msgs
   tf
   eigen_conversions
   kdl_parser
   urdf
   geometry_msgs
   controller_manager
   controller_manager_msgs
   controller_interface
   spacenav_node
   dynamic_reconfigure

   DEPENDS
   gazebo
)

###########
## Build ##
###########
## Declare a C++ library
add_library(${PROJECT_NAME}
  src/panda_kdl.cpp
  src/cartesian_impedance_controller.cpp
  src/spacenav.cpp
  src/polishing_controller.cpp
)

## Add cmake target dependencies of the library
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${catkin_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include
)

## -----------------------------------------------------------------------------
## NODES
## -----------------------------------------------------------------------------
## Declare a C++ executable
add_executable(test_panda_kdl src/nodes/test_panda_kdl.cpp)
add_executable(simulation_spacenav_node src/nodes/simulation_spacenav_node.cpp)
add_executable(simulation_test_node src/nodes/simulation_test_node.cpp)
add_executable(simulation_polishing_node src/nodes/simulation_polishing_node.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(test_panda_kdl ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(simulation_spacenav_node ${catkin_LIBRARIES} ${PROJECT_NAME} sfml-system sfml-window sfml-graphics sfml-audio sfml-network)
target_link_libraries(simulation_test_node ${catkin_LIBRARIES} ${PROJECT_NAME} sfml-system sfml-window sfml-graphics sfml-audio sfml-network)
target_link_libraries(simulation_polishing_node ${catkin_LIBRARIES} ${PROJECT_NAME} sfml-system sfml-window sfml-graphics sfml-audio sfml-network)


## -----------------------------------------------------------------------------


#############
## Install ##
#############
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES panda_controllers_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(
  PROGRAMS scripts/move_to_start.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
