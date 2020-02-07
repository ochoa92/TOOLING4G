cmake_minimum_required(VERSION 2.8.3)
project(franka_spacenav)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -Wall -O3)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  dynamic_reconfigure
  franka_hw
  geometry_msgs
  hardware_interface
  message_generation
  pluginlib
  realtime_tools
  roscpp
  rospy
  tf
  tf_conversions
  actionlib
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(Franka 0.7.0 REQUIRED)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(FILES
  JointTorqueComparison.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages()


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################
## Generate dynamic reconfigure parameters in the 'cfg' folder
#generate_dynamic_reconfigure_options(
#  cfg/compliance_param.cfg
#)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
catkin_package(
  LIBRARIES franka_spacenav
  CATKIN_DEPENDS
    controller_interface
    dynamic_reconfigure
    franka_hw
    geometry_msgs
    hardware_interface
    message_runtime
    pluginlib
    realtime_tools
    roscpp
    tf
    tf_conversions
    actionlib
  DEPENDS Franka
)


###########
## Build ##
###########
## Declare a C++ library
add_library(${PROJECT_NAME}
  src/spacenav.cpp
  src/new_cart_imp_controller.cpp
  src/torque_controller.cpp
  src/cartesian_impedance_controller.cpp
)

## Add cmake target dependencies of the library
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencpp
  #${PROJECT_NAME}_gencfg
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include
)

## ================================= NODES ================================= ##
## Declare a C++ executable
add_executable(spacenav_node src/nodes/spacenav_node.cpp)
add_executable(spacenav_test src/nodes/spacenav_test.cpp)
add_executable(gripper_node src/nodes/gripper_node.cpp)
add_executable(spacenav_pattern_node src/nodes/spacenav_pattern_node.cpp)
add_executable(spacenav_user_node src/nodes/spacenav_user_node.cpp)
add_executable(spacenav_user_pattern_node src/nodes/spacenav_user_pattern_node.cpp)
add_executable(markers_node src/nodes/markers_node.cpp)
add_executable(capsule_node src/nodes/capsule_node.cpp)
add_executable(z_capsule_node src/nodes/z_capsule_node.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(spacenav_node ${catkin_LIBRARIES} ${PROJECT_NAME} sfml-system sfml-window sfml-graphics sfml-audio sfml-network)
target_link_libraries(spacenav_test ${catkin_LIBRARIES} ${PROJECT_NAME} sfml-system sfml-window sfml-graphics sfml-audio sfml-network)
target_link_libraries(gripper_node ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(spacenav_pattern_node ${catkin_LIBRARIES} ${PROJECT_NAME} sfml-system sfml-window sfml-graphics sfml-audio sfml-network)
target_link_libraries(spacenav_user_node ${catkin_LIBRARIES} ${PROJECT_NAME} sfml-system sfml-window sfml-graphics sfml-audio sfml-network)
target_link_libraries(spacenav_user_pattern_node ${catkin_LIBRARIES} ${PROJECT_NAME} sfml-system sfml-window sfml-graphics sfml-audio sfml-network)
target_link_libraries(markers_node ${catkin_LIBRARIES} ${PROJECT_NAME} sfml-system sfml-window sfml-graphics sfml-audio sfml-network)
target_link_libraries(capsule_node ${catkin_LIBRARIES} ${PROJECT_NAME} sfml-system sfml-window sfml-graphics sfml-audio sfml-network)
target_link_libraries(z_capsule_node ${catkin_LIBRARIES} ${PROJECT_NAME} sfml-system sfml-window sfml-graphics sfml-audio sfml-network)


#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES franka_spacenav_controllers_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
catkin_install_python(
  PROGRAMS scripts/franka_states.py
           scripts/interactive_marker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## Tools
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake OPTIONAL
  RESULT_VARIABLE CLANG_TOOLS
)
if(CLANG_TOOLS)
  file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
  file(GLOB_RECURSE HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
  )
  add_format_target(${PROJECT_NAME} FILES ${SOURCES} ${HEADERS})
  add_tidy_target(${PROJECT_NAME}
    FILES ${SOURCES}
    DEPENDS ${PROJECT_NAME}
  )
endif()

#############
## Testing ##
#############
