cmake_minimum_required(VERSION 2.8.3)
project(armin_robothw)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  roscpp
  rospy
  std_msgs
  qt_build
  soem
  moveit_core
  moveit_ros_planning_interface
  controller_interface
  hardware_interface
  controller_manager
    moveit_visual_tools
    pluginlib
    cmake_modules
    geometric_shapes
    joint_limits_interface
    message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Curses )
#find_package(ncurses)

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
add_message_files(
   FILES
     FestoErrors.msg  FestoStatus.msg
     BatteryState.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
   HowDoYouDo.srv
    GetErrorsList.srv
    ExecuteCommand.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages()

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
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
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
  INCLUDE_DIRS include
#  LIBRARIES armin_state_publisher_from_gui
  CATKIN_DEPENDS roscpp rospy std_msgs trajectory_msgs controller_manager hardware_interface 
#  DEPENDS system_lib
)

find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)
set(QT_LIBRARIES Qt5::Widgets)
ADD_DEFINITIONS(${QT_DEFINITIONS})

set(LOCAL_INC ${CMAKE_CURRENT_SOURCE_DIR}/include/armin_robothw )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${LOCAL_INC} ${CMAKE_CURRENT_BINARY_DIR})

add_definitions( -std=c++11 -fPIC)
add_definitions( -DSYNC_PERIOD_MS=10)

#get_cmake_property(_variableNames VARIABLES)
#list (SORT _variableNames)
#foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/armin_robothw/window_first.h include/armin_robothw/joint_setting.h)
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

set( QT_SOURCES
    src/ecn.cpp
    src/ekxx.cpp
    src/elxx.cpp
    src/beckhoff.cpp
    src/el41xx.cpp
    src/festo_over_ec.cpp

    src/joint_setting.cpp
    src/moment_instruction.cpp
    src/qt_based.cpp
    src/window_first.cpp
)

set(CNT_SOURCES
    src/armin_controller_interface.cpp
    src/ecn.cpp
    src/festo_over_ec.cpp
    src/armin_hw.cpp
)

add_executable(armin_state_publisher_from_gui ${QT_SOURCES} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(armin_state_publisher_from_gui
    ${catkin_LIBRARIES}
    ${QT_LIBRARIES}
)

add_executable(armin_control_node
    src/armin_control_node.cpp
    src/armin_hw.cpp
    src/ecn.cpp
    src/festo_over_ec.cpp
    src/ekxx.cpp
    src/elxx.cpp
    src/beckhoff.cpp
    src/el41xx.cpp
)
target_link_libraries(armin_control_node ${catkin_LIBRARIES})
add_dependencies(armin_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(move_group_collision src/move_group_collision.cpp)
target_link_libraries(move_group_collision
    ${catkin_LIBRARIES}
)
add_dependencies(armin_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS})


add_executable(cyclic_move src/cyclic.cpp)
target_link_libraries(cyclic_move
    ${catkin_LIBRARIES}
)

add_executable(set_pdos src/set_pdos.cpp src/ecn.cpp)
target_link_libraries(set_pdos
    ${catkin_LIBRARIES}
)

add_executable(checker src/checker.cpp src/festo_over_ec.cpp src/ecn.cpp)
target_link_libraries(checker
    ${catkin_LIBRARIES}
    ${CURSES_LIBRARIES}
)

add_executable(set_factors src/set_factors.cpp src/ecn.cpp)
target_link_libraries(set_factors
    ${catkin_LIBRARIES}
)

add_executable(prepare_ip_flow src/prepare_ip_flow.cpp src/ecn.cpp)
target_link_libraries(prepare_ip_flow  ${catkin_LIBRARIES} )

if($ENV{ROS_DISTRO} STREQUAL "kinetic")
	add_executable(motion_planning src/motion_planning_api_tutorial.cpp)
	target_link_libraries(motion_planning
    	${catkin_LIBRARIES}
	)
endif()

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/armin_state_publisher_from_gui.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/armin_state_publisher_from_gui_node.cpp)

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

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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

if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(unit_test_robot_hw  tests/test_robot_hw.cpp src/armin_hw.cpp
    src/ecn.cpp
    src/festo_over_ec.cpp
    src/ekxx.cpp
    src/elxx.cpp
    src/beckhoff.cpp
    src/el41xx.cpp
    )

    target_link_libraries(unit_test_robot_hw ${catkin_LIBRARIES})

    catkin_add_gtest(test_elxx_robot_hw  tests/test_elxx_robot_hw.cpp
       src/elxx.cpp  src/beckhoff.cpp src/ecn.cpp
    src/festo_over_ec.cpp
    src/ekxx.cpp
    src/elxx.cpp
    src/beckhoff.cpp
    src/el41xx.cpp
    )
    target_link_libraries(test_elxx_robot_hw ${catkin_LIBRARIES})
endif()

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_armin_state_publisher_from_gui.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
