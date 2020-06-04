# Install script for directory: /home/sjtu_wanghaili/yammar_ws/src

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
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE PROGRAM FILES "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE PROGRAM FILES "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/setup.bash;/usr/local/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/catkin_generated/installspace/setup.bash"
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/setup.sh;/usr/local/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/catkin_generated/installspace/setup.sh"
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/setup.zsh;/usr/local/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/catkin_generated/installspace/setup.zsh"
    "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/catkin_generated/installspace/.rosinstall")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/gtest/cmake_install.cmake")
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/dosuss_srvs/cmake_install.cmake")
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/height_border_msgs/cmake_install.cmake")
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/Qt_gui_test/cmake_install.cmake")
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/avoiding_obstacle/cmake_install.cmake")
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/camera_capture/cmake_install.cmake")
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/control485/cmake_install.cmake")
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/fake_msgs/cmake_install.cmake")
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/full_view/cmake_install.cmake")
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/grain_height_detecte/cmake_install.cmake")
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/harvest_line_detecte/cmake_install.cmake")
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/hmi/cmake_install.cmake")
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/launch_file/cmake_install.cmake")
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/matplotlib_tutorial/cmake_install.cmake")
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/obstacle_detecte/cmake_install.cmake")
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/obstacle_detecte_fake/cmake_install.cmake")
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/path_control/cmake_install.cmake")
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/path_planning/cmake_install.cmake")
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/reap_height_capture/cmake_install.cmake")
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/reap_height_control/cmake_install.cmake")
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/reap_unit_control/cmake_install.cmake")
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/system_monitor/cmake_install.cmake")
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/action_tutorials/cmake_install.cmake")
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/pnp_msgs/cmake_install.cmake")
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/pnp_ros/cmake_install.cmake")
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/reap_height_action/cmake_install.cmake")
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/reap_unit_action/cmake_install.cmake")
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/rp_action_msgs/cmake_install.cmake")
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/controlcan/cmake_install.cmake")
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/birdView_calibration_v2/cmake_install.cmake")
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/boud_rgbd/cmake_install.cmake")
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/camera_image/cmake_install.cmake")
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/find_cut/cmake_install.cmake")
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/mmw_capture/cmake_install.cmake")
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/rqt_image_view/cmake_install.cmake")
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/smach_tutorials/cmake_install.cmake")
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/rp_demo/cmake_install.cmake")
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/rp_pnp/cmake_install.cmake")
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/travel_control/cmake_install.cmake")
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/unload_position_detecte/cmake_install.cmake")
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/usb_cam/cmake_install.cmake")
  include("/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/rp_action/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
