cmake_minimum_required(VERSION 2.8.3)
project(balance_controller)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(ct_core_DIR /usr/local/share/ct_core/cmake)
set(ct_models_DIR /usr/local/share/ct_models/cmake)
set(ct_optcon_DIR /usr/local/share/ct_optcon/cmake)
set(ct_rbd_DIR /usr/local/share/ct_rbd/cmake)

find_package(catkin REQUIRED COMPONENTS
  qp_solver
  roscpp
  free_gait_core
  quadruped_model
  quadruped_odom
  free_gait_ros
  controller_interface
  pluginlib
  gazebo_dev
  control_toolbox
  controller_manager
  hardware_interface
  transmission_interface
  urdf
  joint_limits_interface
  gazebo_ros_control
  forward_command_controller
  realtime_tools
  sim_assiants
  ooqp_eigen_interface
  single_leg_test
  ros_ethercat_driver
  dynamic_reconfigure
  rosbag
  grid_map_core
  ct_core
  ct_models
  ct_optcon
  ct_rbd
)

## System dependencies are found with CMake's conventions
 find_package(Boost REQUIRED QUIET)
 find_package(ifopt 2.0.1 REQUIRED)
 find_package(Eigen3 REQUIRED)


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
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
 generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
  cfg/controller_gains.cfg
 )

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
  INCLUDE_DIRS include
  LIBRARIES balance_controller gazebo_state_hardware_interface robot_state_gazebo_ros_control_plugin base_motion_planning
  CATKIN_DEPENDS
  qp_solver
  roscpp
  free_gait_core
  quadruped_model
  quadruped_odom
  free_gait_ros
  controller_interface
  pluginlib
  gazebo_dev
  control_toolbox
  controller_manager
  hardware_interface
  transmission_interface
  urdf
  joint_limits_interface
  gazebo_ros_control
  forward_command_controller
  realtime_tools
  sim_assiants
  ooqp_eigen_interface
  rosbag
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  test/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${ifopt_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
   src/motion_control/MotionControllerBase.cpp
   src/motion_control/VirtualModelController.cpp
   src/contact_force_distribution/ContactForceDistributionBase.cpp
   src/contact_force_distribution/ContactForceDistribution.cpp
   src/contact_force_distribution/ContactJointTorqueDistribution.cpp
   src/state_switcher/StateSwitcher.cpp
   src/state_switcher/StateSwitcherBase.cpp
   src/quadruped_model_CPPad/Quadruped_Kin_CppAD.cpp
#   src/ros_controller/ros_balance_controller.cpp
#   src/ros_controller/gazebo_state_hardware_interface.cpp
#   include/balance_controller/contact_force_distribution/ContactForceDistributionBase.hpp
 )
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARY_DIRS}
    ${ifopt_LIBRARIES}
    /usr/local/lib/libiitgenquadruped_model.so
    /usr/local/lib/libiitgenhyq.so)

add_library(gazebo_state_hardware_interface
    src/ros_controller/gazebo_state_hardware_interface.cpp
  )
target_link_libraries(gazebo_state_hardware_interface ${catkin_LIBRARIES})

add_library(ros_balance_controller
  src/ros_controller/ros_balance_controller.cpp
  )
target_link_libraries(ros_balance_controller balance_controller ${catkin_LIBRARIES})

add_library(robot_state_gazebo_ros_control_plugin
    src/ros_controller/robot_state_gazebo_ros_control_plugin.cpp
  )
target_link_libraries(robot_state_gazebo_ros_control_plugin
  ${catkin_LIBRARIES}
  gazebo_state_hardware_interface
  )

add_library(base_motion_planning
    src/base_motion_planning/base_motion_planning.cpp
  )
target_link_libraries(base_motion_planning
  ${catkin_LIBRARIES}
  )

add_library(state_estimate_controller
  src/ros_controller/state_estimate_controller.cpp
  )
target_link_libraries(state_estimate_controller
  ${catkin_LIBRARIES}
  )

add_library(single_leg_controller
src/ros_controller/single_leg_controller.cpp
)
target_link_libraries(single_leg_controller
${catkin_LIBRARIES}
)

add_library(static_walk_controller
src/ros_controller/static_walk_controller.cpp
)

target_link_libraries(static_walk_controller
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARY_DIRS}
    ${ifopt_LIBRARIES}
    /usr/local/lib/libiitgenquadruped_model.so
    /usr/local/lib/libiitgenhyq.so
)

add_library(ifopt_test_lib
test/ifopt_test.cpp
)

target_link_libraries(ifopt_test_lib
${catkin_LIBRARIES}
${ifopt_LIBRARIES}
)

add_library(quadruped_nonlinear_test_lib
test/lib/quadruped_nonlinear_test.cpp
)

target_link_libraries(quadruped_nonlinear_test_lib
${catkin_LIBRARIES}
${ifopt_LIBRARIES}
)

#add_library(joint_torque_controller src/ros_controller/joint_torque_controller.cpp)
#target_link_libraries(joint_torque_controller
#  ${catkin_LIBRARIES}
#  )


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/balance_controller_node.cpp)
add_executable(balance_controller_test_node test/test.cpp)
add_executable(balance_controller_manager src/ros_controller/balance_controller_manager.cpp)
add_executable(read_rosbag_file_test_node test/read_rosbag_file_test.cpp)
target_link_libraries(read_rosbag_file_test_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARY_DIRS}
  )
add_executable(test_force_distribution_kp_node test/test_force_distribution_kp.cpp)
target_link_libraries(test_force_distribution_kp_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARY_DIRS}
  )

add_executable(base_motion_planning_node test/base_motion_planning_node.cpp)
target_link_libraries(base_motion_planning_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARY_DIRS}
  base_motion_planning
  )

add_executable(ifopt_test_execuable_node test/ifopt_test_execuable.cpp)
target_link_libraries(ifopt_test_execuable_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARY_DIRS}
  ${ifopt_LIBRARIES}
  /usr/local/lib/libiitgenquadruped_model.so
  /usr/local/lib/libiitgenhyq.so
  )

add_executable(Quadruped_model_cppad_test_node test/Quadruped_model_cppad_test.cpp)
target_link_libraries(Quadruped_model_cppad_test_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARY_DIRS}
  ${ifopt_LIBRARIES}
  /usr/local/lib/libiitgenquadruped_model.so
  /usr/local/lib/libiitgenhyq.so
  )

add_executable(optimization_server_node test/optimization_server.cpp)
target_link_libraries(optimization_server_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARY_DIRS}
  ${ifopt_LIBRARIES}
  /usr/local/lib/libiitgenquadruped_model.so
  /usr/local/lib/libiitgenhyq.so
  )



add_executable(quadruped_nonlinear_exe_node test/quadruped_nonlinear_exe.cpp)
target_link_libraries(quadruped_nonlinear_exe_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARY_DIRS}
  ${ifopt_LIBRARIES}
  /usr/local/lib/libiitgenquadruped_model.so
  )



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
#   ${catkin_LIBRARIES}
# )
target_link_libraries(balance_controller_test_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )
target_link_libraries(balance_controller_manager
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )
add_library(configure_change_controller
    src/ros_controller/configure_change_controller.cpp
    )
target_link_libraries(configure_change_controller ${PROJECT_NAME} ${catkin_LIBRARIES})

add_library(walk_forward_controller
    src/ros_controller/walk_forward_controller.cpp
    )
target_link_libraries(walk_forward_controller ${PROJECT_NAME} ${catkin_LIBRARIES})


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
 install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.hpp"
#   PATTERN ".svn" EXCLUDE
 )

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
 catkin_add_gtest(${PROJECT_NAME}-test
   test/test_balance_controller.cpp
   test/virtual_model_controller_test.cpp
   )
 if(TARGET ${PROJECT_NAME}-test)
   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES})
 endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
