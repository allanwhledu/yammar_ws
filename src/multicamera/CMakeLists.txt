#1---------------------------> Basic env. setting
cmake_minimum_required(VERSION 3.2)
project(multicamera)
add_definitions(-std=c++11)                                          #specify compiler

set(SOURCE_PATH ${PROJECT_SOURCE_DIR}/src)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  tf
  message_generation
  cv_bridge
  image_transport
#  beginner_tutorials
)


#opencv
find_package(OpenCV REQUIRED)
#BOOST
set(BOOST_LIBS program_options serialization random system)
find_package(Boost 1.48.0 COMPONENTS ${BOOST_LIBS} REQUIRED)
MESSAGE("Boost information:") 
MESSAGE("  Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}") 
MESSAGE("  Boost_LIBRARIES: ${Boost_LIBRARIES}") 
MESSAGE("  Boost_LIBRARY_DIRS: ${Boost_LIBRARY_DIRS}")
#add_service_files(
#  FILES
#  smooth_srv.srv
#  smooth_srv_multi.srv
#)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

catkin_package(
INCLUDE_DIRS include
CATKIN_DEPENDS  std_msgs sensor_msgs geometry_msgs tf message_runtime
)


include_directories(
 ${PROJECT_SOURCE_DIR}/include
 ${catkin_INCLUDE_DIRS}
 ${Boost_INCLUDE_DIRS}
)

LINK_DIRECTORIES(
 ${PROJECT_SOURCE_DIR}/lib
　${Boost_LIBRARY_DIRS}
 "usr/local/lib"
)

#2--------------------------->  Link lib and include

#3--------------------------->  Create executeable file
add_executable(multicamera_realtime ${SOURCE_PATH}/main_realtime.cpp)
target_link_libraries(multicamera_realtime 
	${catkin_LIBRARIES} 
	${OpenCV_LIBS} 
        ${Boost_LIBRARIES}
	libMVSDK.so 
	pthread
)

add_executable(multicamera ${SOURCE_PATH}/main.cpp)
target_link_libraries(multicamera 
	${catkin_LIBRARIES} 
	${OpenCV_LIBS} 
   	${Boost_LIBRARIES}
	libMVSDK.so 
	pthread
)
