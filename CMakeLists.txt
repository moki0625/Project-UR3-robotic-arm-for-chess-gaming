cmake_minimum_required(VERSION 2.8.3)
project(team04_final_project)

add_definitions(-std=c++11 -Wall -Wextra -Wdeprecated)#-Wpedantic -Werror)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  aruco_ros
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  message_generation
  chesslab_setup
  actionlib_msgs
)

find_package( Eigen3 REQUIRED )

include_directories(${catkin_INCLUDE_DIRS} include)

## Generate service in the 'srv' folder
add_service_files(
  FILES
  getpiecepose.srv
  getFieldpose.srv
  getOccupiedFields.srv
  getpiececellcode.srv
  transfercoordinate.srv
  moveArm.srv
  pickChess.srv
  placeChess.srv
  chessCommand.srv
  optik.srv
  pregraspToBase.srv
)

## Generate messages in the 'msg' folder
add_message_files(
   DIRECTORY msg
   FILES
   ik.msg
   occupiedFields.msg
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  robot_action.action
)

## Generate added messages and services with any dependencies listed here
add_definitions( "--std=gnu++11" )
generate_messages(
  DEPENDENCIES
  roscpp
  std_msgs
  geometry_msgs
  trajectory_msgs
  actionlib_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS  ${CMAKE_SOURCE_DIR}
  CATKIN_DEPENDS roscpp gazebo_ros aruco_ros std_msgs tf2 tf2_ros geometry_msgs rospy trajectory_msgs control_msgs actionlib actionlib_msgs message_runtime chesslab_setup
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_executable(sensing_module src/sensing_module.cpp)
add_executable(planning_module src/planning_module.cpp)
add_executable(chess_manager src/chess_manager.cpp)
add_executable(action_module src/action_module.cpp)


target_link_libraries(sensing_module ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(planning_module ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(chess_manager ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(action_module ${catkin_LIBRARIES})

add_dependencies(sensing_module ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(planning_module ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(chess_manager ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(action_module ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
