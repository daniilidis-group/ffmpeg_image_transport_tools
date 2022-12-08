cmake_minimum_required(VERSION 3.5)
project(ffmpeg_image_transport_tools)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wall")

find_package(catkin REQUIRED COMPONENTS roscpp nodelet rosbag cv_bridge
image_transport ffmpeg_image_transport_msgs ffmpeg_image_transport
sensor_msgs flex_sync)

catkin_package(CATKIN_DEPENDS
roscpp
nodelet
rosbag
flex_sync
ffmpeg_image_transport_msgs
ffmpeg_image_transport
sensor_msgs)

find_package(OpenCV)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})

# first the base libary for play_bag
add_library(play_bag src/play_bag.cpp)
target_link_libraries(play_bag ${catkin_LIBRARIES})

# then the nodelet library, linked against it
add_library(play_bag_nodelet src/play_bag_nodelet.cpp)
target_link_libraries(play_bag_nodelet play_bag ${catkin_LIBRARIES})

# executables

add_executable(play_bag_node src/play_bag_node.cpp )
target_link_libraries(play_bag_node play_bag ${catkin_LIBRARIES}
)

add_executable(decode_bag src/decode_bag.cpp src/decode_bag_node.cpp)
target_link_libraries(decode_bag ${catkin_LIBRARIES}
)

add_executable(split_bag src/split_bag.cpp src/split_bag_node.cpp)
target_link_libraries(split_bag ${catkin_LIBRARIES}
)

add_executable(decode_frames src/decode_frames.cpp src/decode_frames_node.cpp)
target_link_libraries(decode_frames ${catkin_LIBRARIES}
)
