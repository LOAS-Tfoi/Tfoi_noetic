cmake_minimum_required(VERSION 3.0.2)
project(service_call_server)

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

add_service_files(
  FILES
  RobotTurning.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
)



catkin_package(
  # INCLUDE_DIRS include
  #LIBRARIES ros_opencv
  CATKIN_DEPENDS cv_bridge roscpp sensor_msgs nav_msgs tf actionlib move_base_msgs message_generation
  #  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(service_call_server_node src/service_call_server.cpp)
target_link_libraries(service_call_server_node ${catkin_LIBRARIES})
