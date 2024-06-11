cmake_minimum_required(VERSION 3.0.2)
project(recharge_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  sensor_msgs
  nav_msgs
  tf
  geometry_msgs
  actionlib
  move_base_msgs
  message_generation
)
  

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package(Eigen REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})

add_message_files(
  # ChassisInfo.msg
  # ctrl_cmd.msg
  # ctrl_fb.msg  
)

add_service_files(
  FILES
  PathIsVaild.srv
  DisRecharge.srv
  Recharge.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
)

catkin_package(
  # INCLUDE_DIRS include
  #LIBRARIES ros_opencv
  CATKIN_DEPENDS cv_bridge roscpp sensor_msgs nav_msgs tf actionlib move_base_msgs message_runtime
  #  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include 
)

add_executable(recharge_controller_node src/recharge_controller_node.cpp)
target_link_libraries(recharge_controller_node ${catkin_LIBRARIES})
