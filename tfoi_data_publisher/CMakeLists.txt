cmake_minimum_required(VERSION 3.0.2)
project(tfoi_data_publisher)

#set(CMAKE_C++ C_STARNDARD 14)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  std_msgs
  #cv_bridge  
  message_generation
  tf
  tf_conversions 
)

find_package(OpenCV REQUIRED)
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_message_files(
  FILES
  ultrasonic.msg
  bms_fb.msg
  bms_flag_fb.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  #INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
)

add_executable(tfoi_data_publisher_node src/tfoi_data_publisher_node.cpp src/tfoi_data_publisher.cpp)
target_link_libraries(tfoi_data_publisher_node ${catkin_LIBRARIES}
${catkin_LIBRARIES}
${OpenCV_LIBRARIES}
)
