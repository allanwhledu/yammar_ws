# Install script for directory: /home/sjtu_wanghaili/yammar_ws/src/action_tutorials

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/action_tutorials/action" TYPE FILE FILES "/home/sjtu_wanghaili/yammar_ws/src/action_tutorials/action/DoDishes.action")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/action_tutorials/msg" TYPE FILE FILES
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/action_tutorials/msg/DoDishesAction.msg"
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/action_tutorials/msg/DoDishesActionGoal.msg"
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/action_tutorials/msg/DoDishesActionResult.msg"
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/action_tutorials/msg/DoDishesActionFeedback.msg"
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/action_tutorials/msg/DoDishesGoal.msg"
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/action_tutorials/msg/DoDishesResult.msg"
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/action_tutorials/msg/DoDishesFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/action_tutorials/cmake" TYPE FILE FILES "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/action_tutorials/catkin_generated/installspace/action_tutorials-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/include/action_tutorials")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/roseus/ros/action_tutorials")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/common-lisp/ros/action_tutorials")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/gennodejs/ros/action_tutorials")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/lib/python2.7/dist-packages/action_tutorials")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/lib/python2.7/dist-packages/action_tutorials")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/action_tutorials/catkin_generated/installspace/action_tutorials.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/action_tutorials/cmake" TYPE FILE FILES "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/action_tutorials/catkin_generated/installspace/action_tutorials-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/action_tutorials/cmake" TYPE FILE FILES
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/action_tutorials/catkin_generated/installspace/action_tutorialsConfig.cmake"
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/action_tutorials/catkin_generated/installspace/action_tutorialsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/action_tutorials" TYPE FILE FILES "/home/sjtu_wanghaili/yammar_ws/src/action_tutorials/package.xml")
endif()

