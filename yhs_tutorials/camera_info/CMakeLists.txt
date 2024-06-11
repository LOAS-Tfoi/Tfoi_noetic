cmake_minimum_required(VERSION 3.0.2)
project(camera_info)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
)
find_package(OpenCV REQUIRED)


catkin_package(

)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_LIBRARIES}
)

add_executable(camera_info src/camera_info.cpp)
target_link_libraries(camera_info ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES} )
