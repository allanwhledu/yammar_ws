cmake_minimum_required(VERSION 2.8.3)
project(reap_height_capture)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        )


include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/include
        )

add_executable(${PROJECT_NAME}_node src/reap_height_capture_node.cpp src/CAN_DEVICE.cpp src/CAN_DEVICE.h)

target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/shared_lib/libcontrolcan.so
        )
