# Install script for directory: /home/sjtu_wanghaili/yammar_ws/src/reap_unit_control

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/reap_unit_control/action" TYPE FILE FILES "/home/sjtu_wanghaili/yammar_ws/src/reap_unit_control/action/ControlReap.action")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/reap_unit_control/msg" TYPE FILE FILES
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/reap_unit_control/msg/ControlReapAction.msg"
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/reap_unit_control/msg/ControlReapActionGoal.msg"
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/reap_unit_control/msg/ControlReapActionResult.msg"
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/reap_unit_control/msg/ControlReapActionFeedback.msg"
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/reap_unit_control/msg/ControlReapGoal.msg"
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/reap_unit_control/msg/ControlReapResult.msg"
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/reap_unit_control/msg/ControlReapFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/reap_unit_control/cmake" TYPE FILE FILES "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/reap_unit_control/catkin_generated/installspace/reap_unit_control-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/include/reap_unit_control")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/roseus/ros/reap_unit_control")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/common-lisp/ros/reap_unit_control")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/gennodejs/ros/reap_unit_control")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/lib/python2.7/dist-packages/reap_unit_control")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/lib/python2.7/dist-packages/reap_unit_control")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/reap_unit_control/catkin_generated/installspace/reap_unit_control.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/reap_unit_control/cmake" TYPE FILE FILES "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/reap_unit_control/catkin_generated/installspace/reap_unit_control-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/reap_unit_control/cmake" TYPE FILE FILES
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/reap_unit_control/catkin_generated/installspace/reap_unit_controlConfig.cmake"
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/reap_unit_control/catkin_generated/installspace/reap_unit_controlConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/reap_unit_control" TYPE FILE FILES "/home/sjtu_wanghaili/yammar_ws/src/reap_unit_control/package.xml")
endif()

