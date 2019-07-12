cmake_minimum_required(VERSION 2.8.3)
project(franka_polishing)

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
  franka_msgs
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(Franka 0.5.0 REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
# )

## Generate added messages and services with any dependencies listed here
#generate_messages()

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  LIBRARIES franka_polishing
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
    franka_msgs
  DEPENDS Franka
)

###########
## Build ##
###########
## Declare a C++ library
add_library(${PROJECT_NAME}
  src/cartesian_impedance_controller.cpp
  src/spacenav.cpp
  src/training_mode.cpp
  src/co_manipulation_mode.cpp
)

## Add cmake target dependencies of the library
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencpp
#  ${PROJECT_NAME}_gencfg
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
add_executable(polishing_test_node src/nodes/polishing_test_node.cpp)
add_executable(polishing_spacenav_node src/nodes/polishing_spacenav_node.cpp)
add_executable(user_pattern_node src/nodes/user_pattern_node.cpp)
add_executable(flat_surface_node src/nodes/flat_surface_node.cpp)
add_executable(nflat_surfaces_node src/nodes/nflat_surfaces_node.cpp)
add_executable(curved_surface_node src/nodes/curved_surface_node.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(polishing_test_node ${catkin_LIBRARIES} ${PROJECT_NAME} sfml-system sfml-window sfml-graphics sfml-audio sfml-network)
target_link_libraries(polishing_spacenav_node ${catkin_LIBRARIES} ${PROJECT_NAME} sfml-system sfml-window sfml-graphics sfml-audio sfml-network)
target_link_libraries(user_pattern_node ${catkin_LIBRARIES} ${PROJECT_NAME} sfml-system sfml-window sfml-graphics sfml-audio sfml-network)
target_link_libraries(flat_surface_node ${catkin_LIBRARIES} ${PROJECT_NAME} sfml-system sfml-window sfml-graphics sfml-audio sfml-network)
target_link_libraries(nflat_surfaces_node ${catkin_LIBRARIES} ${PROJECT_NAME} sfml-system sfml-window sfml-graphics sfml-audio sfml-network)
target_link_libraries(curved_surface_node ${catkin_LIBRARIES} ${PROJECT_NAME} sfml-system sfml-window sfml-graphics sfml-audio sfml-network)


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
install(FILES franka_polishing_controllers_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
#catkin_install_python(
#  PROGRAMS scripts/franka_states.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)


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

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_franka_polishing.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
