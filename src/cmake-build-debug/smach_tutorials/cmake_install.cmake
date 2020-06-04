# Install script for directory: /home/sjtu_wanghaili/yammar_ws/src/smach_tutorials

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/smach_tutorials/action" TYPE FILE FILES "/home/sjtu_wanghaili/yammar_ws/src/smach_tutorials/action/Test.action")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/smach_tutorials/msg" TYPE FILE FILES
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/smach_tutorials/msg/TestAction.msg"
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/smach_tutorials/msg/TestActionGoal.msg"
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/smach_tutorials/msg/TestActionResult.msg"
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/smach_tutorials/msg/TestActionFeedback.msg"
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/smach_tutorials/msg/TestGoal.msg"
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/smach_tutorials/msg/TestResult.msg"
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/smach_tutorials/msg/TestFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/smach_tutorials/cmake" TYPE FILE FILES "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/smach_tutorials/catkin_generated/installspace/smach_tutorials-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/include/smach_tutorials")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/roseus/ros/smach_tutorials")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/common-lisp/ros/smach_tutorials")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/gennodejs/ros/smach_tutorials")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/lib/python2.7/dist-packages/smach_tutorials")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/lib/python2.7/dist-packages/smach_tutorials")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/smach_tutorials/catkin_generated/installspace/smach_tutorials.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/smach_tutorials/cmake" TYPE FILE FILES "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/smach_tutorials/catkin_generated/installspace/smach_tutorials-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/smach_tutorials/cmake" TYPE FILE FILES
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/smach_tutorials/catkin_generated/installspace/smach_tutorialsConfig.cmake"
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/smach_tutorials/catkin_generated/installspace/smach_tutorialsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/smach_tutorials" TYPE FILE FILES "/home/sjtu_wanghaili/yammar_ws/src/smach_tutorials/package.xml")
endif()

