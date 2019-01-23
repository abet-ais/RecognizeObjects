cmake_minimum_required(VERSION 2.8.3)
project(abe_gazebo_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  tf
  pcl_conversions
  pcl_ros
  sensor_msgs
  geometry_msgs
  std_srvs
  image_transport
  cv_bridge
)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(OpenCV REQUIRED)
INCLUDE(FindPkgConfig)
pkg_check_modules(OPENCV REQUIRED opencv)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
# INCLUDE(FindPkgConfig)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

# Generate services in the 'srv' folder
add_service_files(DIRECTORY srv
  FILES
  set_service.srv
  gazebo_pose.srv
  gazebo_obj_pose.srv
  createTemp_switch.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES abe_gazebo_planner
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
 DEPENDS system_lib
)
 
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/abe_gazebo_planner.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/abe_gazebo_planner_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
  # ${catkin_LIBRARIES}
# )


# add_executable(turtlebot_crient src/turtlebot_crient.cpp)
# target_link_libraries(turtlebot_crient ${catkin_LIBRARIES})
# add_dependencies(turtlebot_crient abe_sandbox_gencpp)

# add_executable(turtlebot_crient2 src/turtlebot_crient2.cpp)
# target_link_libraries(turtlebot_crient2 ${catkin_LIBRARIES})
# add_dependencies(turtlebot_crient2 abe_sandbox_gencpp)

# add_executable(keypoints src/keypoints.cpp)
# target_link_libraries(keypoints ${catkin_LIBRARIES} ${OPENCV_LIBRARIES})
# add_dependencies(keypoints abe_sandbox_gencpp)

# add_executable(keypoints_csv src/keypoints_csv.cpp)
# target_link_libraries(keypoints_csv ${catkin_LIBRARIES} ${OPENCV_LIBRARIES})
# add_dependencies(keypoints_csv abe_sandbox_gencpp)

# add_executable(openpose_depth src/openpose_depth.cpp)
# target_link_libraries(openpose_depth ${catkin_LIBRARIES} ${OPENCV_LIBRARIES})
# add_dependencies(openpose_depth abe_sandbox_gencpp)

# add_executable(cvmat_player src/cvmat_player.cpp)
# target_link_libraries(cvmat_player ${catkin_LIBRARIES} ${OPENCV_LIBRARIES})
# add_dependencies(cvmat_player abe_sandbox_gencpp)

# add_executable(abe_get_pose_video src/get_pose_video.cpp)
# target_link_libraries(abe_get_pose_video ${catkin_LIBRARIES} ${OPENCV_LIBRARIES})

# add_executable(viewpoint_planner src/viewpoint_planner.cpp)
# target_link_libraries(viewpoint_planner ${catkin_LIBRARIES} ${OPENCV_LIBRARIES})
# add_dependencies(viewpoint_planner abe_sandbox_gencpp)

add_executable(createTemp_TB src/createTemp_TB.cpp)
target_link_libraries(createTemp_TB ${catkin_LIBRARIES} ${OPENCV_LIBRARIES})

add_executable(createTemp_Ki src/createTemp_Ki.cpp)
target_link_libraries(createTemp_Ki ${catkin_LIBRARIES} ${OPENCV_LIBRARIES})

add_executable(createTemp_min src/createTemp_min.cpp)
target_link_libraries(createTemp_min ${catkin_LIBRARIES} ${OPENCV_LIBRARIES})

# add_executable(json_perser src/json_perser.cpp)
# target_link_libraries(json_perser ${catkin_LIBRARIES})
# add_dependencies(json_perser abe_sandbox_gencpp)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_abe_gazebo_planner.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
