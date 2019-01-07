cmake_minimum_required(VERSION 2.8)
project(ffmpeg_image_transport_tools)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wall")

find_package(catkin REQUIRED COMPONENTS roscpp rosbag cv_bridge
image_transport ffmpeg_image_transport_msgs ffmpeg_image_transport
sensor_msgs flex_sync)

catkin_package(CATKIN_DEPENDS
roscpp
rosbag
flex_sync
ffmpeg_image_transport_msgs
ffmpeg_image_transport
sensor_msgs)

find_package(OpenCV)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})

# add the plugin
add_executable(decode_bag src/decode_bag.cpp src/decode_bag_node.cpp)
target_link_libraries(decode_bag ${catkin_LIBRARIES}
)
