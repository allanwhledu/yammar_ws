# Install script for directory: /home/sjtu_wanghaili/yammar_ws/src/reap_unit_action

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/reap_unit_action/action" TYPE FILE FILES "/home/sjtu_wanghaili/yammar_ws/src/reap_unit_action/action/ControlReap.action")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/reap_unit_action/msg" TYPE FILE FILES
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/reap_unit_action/msg/ControlReapAction.msg"
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/reap_unit_action/msg/ControlReapActionGoal.msg"
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/reap_unit_action/msg/ControlReapActionResult.msg"
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/reap_unit_action/msg/ControlReapActionFeedback.msg"
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/reap_unit_action/msg/ControlReapGoal.msg"
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/reap_unit_action/msg/ControlReapResult.msg"
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/reap_unit_action/msg/ControlReapFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/reap_unit_action/cmake" TYPE FILE FILES "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/reap_unit_action/catkin_generated/installspace/reap_unit_action-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/include/reap_unit_action")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/roseus/ros/reap_unit_action")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/common-lisp/ros/reap_unit_action")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/gennodejs/ros/reap_unit_action")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/lib/python2.7/dist-packages/reap_unit_action")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/lib/python2.7/dist-packages/reap_unit_action")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/reap_unit_action/catkin_generated/installspace/reap_unit_action.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/reap_unit_action/cmake" TYPE FILE FILES "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/reap_unit_action/catkin_generated/installspace/reap_unit_action-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/reap_unit_action/cmake" TYPE FILE FILES
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/reap_unit_action/catkin_generated/installspace/reap_unit_actionConfig.cmake"
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/reap_unit_action/catkin_generated/installspace/reap_unit_actionConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/reap_unit_action" TYPE FILE FILES "/home/sjtu_wanghaili/yammar_ws/src/reap_unit_action/package.xml")
endif()

