cmake_minimum_required(VERSION 2.8.3)
project(learn)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(talker talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})


add_executable(listener listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})


add_executable(pid pid.cpp)
target_link_libraries(pid ${catkin_LIBRARIES})

