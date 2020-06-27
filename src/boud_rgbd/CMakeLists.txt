cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_STANDARD 11)
project(boud_rgbd)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        pcl_ros
        cv_bridge
        image_transport

        std_msgs
        height_border_msgs
        )
catkin_package(CATKIN_DEPENDS height_border_msgs)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
find_package(Eigen3 REQUIRED)
include_directories( ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})

add_executable(boud_rgbd main.cpp)
target_link_libraries(boud_rgbd ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
