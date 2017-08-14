# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(ja_robbie_cam)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg message_generation sensor_msgs cv_bridge image_transport)

## Declare ROS messages and services
add_message_files(FILES Num.msg)
add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(
CATKIN_DEPENDS message_runtime
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(camera_feed src/camera_feed.cpp)
target_link_libraries(camera_feed ${catkin_LIBRARIES})
add_dependencies(camera_feed ja_robbie_cam_generate_messages_cpp)

add_executable(camera_feed_listen src/camera_feed_listen.cpp)
target_link_libraries(camera_feed_listen ${catkin_LIBRARIES})
add_dependencies(camera_feed_listen ja_robbie_cam_generate_messages_cpp)

# %EndTag(FULLTEXT)%

