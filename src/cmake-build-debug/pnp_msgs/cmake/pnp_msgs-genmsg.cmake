# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "pnp_msgs: 9 messages, 6 services")

set(MSG_I_FLAGS "-Ipnp_msgs:/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/msg;-Ipnp_msgs:/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg;-Iactionlib_msgs:/opt/ros/melodic/share/actionlib_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(pnp_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPLastEvent.srv" NAME_WE)
add_custom_target(_pnp_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pnp_msgs" "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPLastEvent.srv" ""
)

get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPSetVariableValue.srv" NAME_WE)
add_custom_target(_pnp_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pnp_msgs" "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPSetVariableValue.srv" ""
)

get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPCondition.srv" NAME_WE)
add_custom_target(_pnp_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pnp_msgs" "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPCondition.srv" ""
)

get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionResult.msg" NAME_WE)
add_custom_target(_pnp_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pnp_msgs" "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionResult.msg" "actionlib_msgs/GoalID:pnp_msgs/PNPResult:actionlib_msgs/GoalStatus:std_msgs/Header"
)

get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPAction.msg" NAME_WE)
add_custom_target(_pnp_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pnp_msgs" "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPAction.msg" "actionlib_msgs/GoalID:pnp_msgs/PNPResult:actionlib_msgs/GoalStatus:pnp_msgs/PNPActionResult:pnp_msgs/PNPGoal:pnp_msgs/PNPActionFeedback:std_msgs/Header:pnp_msgs/PNPFeedback:pnp_msgs/PNPActionGoal"
)

get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionGoal.msg" NAME_WE)
add_custom_target(_pnp_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pnp_msgs" "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionGoal.msg" "actionlib_msgs/GoalID:pnp_msgs/PNPGoal:std_msgs/Header"
)

get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/msg/Action.msg" NAME_WE)
add_custom_target(_pnp_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pnp_msgs" "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/msg/Action.msg" ""
)

get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPGetVariableValue.srv" NAME_WE)
add_custom_target(_pnp_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pnp_msgs" "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPGetVariableValue.srv" ""
)

get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPFeedback.msg" NAME_WE)
add_custom_target(_pnp_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pnp_msgs" "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPFeedback.msg" ""
)

get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionFeedback.msg" NAME_WE)
add_custom_target(_pnp_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pnp_msgs" "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionFeedback.msg" "actionlib_msgs/GoalID:actionlib_msgs/GoalStatus:pnp_msgs/PNPFeedback:std_msgs/Header"
)

get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPGoal.msg" NAME_WE)
add_custom_target(_pnp_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pnp_msgs" "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPGoal.msg" ""
)

get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/msg/ActionFinished.msg" NAME_WE)
add_custom_target(_pnp_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pnp_msgs" "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/msg/ActionFinished.msg" ""
)

get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPResult.msg" NAME_WE)
add_custom_target(_pnp_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pnp_msgs" "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPResult.msg" ""
)

get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPReward.srv" NAME_WE)
add_custom_target(_pnp_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pnp_msgs" "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPReward.srv" ""
)

get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPClearBuffer.srv" NAME_WE)
add_custom_target(_pnp_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pnp_msgs" "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPClearBuffer.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pnp_msgs
)
_generate_msg_cpp(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionResult.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPGoal.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPFeedback.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pnp_msgs
)
_generate_msg_cpp(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pnp_msgs
)
_generate_msg_cpp(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/msg/Action.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pnp_msgs
)
_generate_msg_cpp(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pnp_msgs
)
_generate_msg_cpp(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pnp_msgs
)
_generate_msg_cpp(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pnp_msgs
)
_generate_msg_cpp(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/msg/ActionFinished.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pnp_msgs
)
_generate_msg_cpp(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pnp_msgs
)

### Generating Services
_generate_srv_cpp(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPSetVariableValue.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pnp_msgs
)
_generate_srv_cpp(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPCondition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pnp_msgs
)
_generate_srv_cpp(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPReward.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pnp_msgs
)
_generate_srv_cpp(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPGetVariableValue.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pnp_msgs
)
_generate_srv_cpp(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPLastEvent.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pnp_msgs
)
_generate_srv_cpp(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPClearBuffer.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pnp_msgs
)

### Generating Module File
_generate_module_cpp(pnp_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pnp_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(pnp_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(pnp_msgs_generate_messages pnp_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPLastEvent.srv" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_cpp _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPSetVariableValue.srv" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_cpp _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPCondition.srv" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_cpp _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionResult.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_cpp _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPAction.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_cpp _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionGoal.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_cpp _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/msg/Action.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_cpp _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPGetVariableValue.srv" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_cpp _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPFeedback.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_cpp _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionFeedback.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_cpp _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPGoal.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_cpp _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/msg/ActionFinished.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_cpp _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPResult.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_cpp _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPReward.srv" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_cpp _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPClearBuffer.srv" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_cpp _pnp_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pnp_msgs_gencpp)
add_dependencies(pnp_msgs_gencpp pnp_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pnp_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pnp_msgs
)
_generate_msg_eus(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionResult.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPGoal.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPFeedback.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pnp_msgs
)
_generate_msg_eus(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pnp_msgs
)
_generate_msg_eus(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/msg/Action.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pnp_msgs
)
_generate_msg_eus(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pnp_msgs
)
_generate_msg_eus(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pnp_msgs
)
_generate_msg_eus(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pnp_msgs
)
_generate_msg_eus(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/msg/ActionFinished.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pnp_msgs
)
_generate_msg_eus(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pnp_msgs
)

### Generating Services
_generate_srv_eus(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPSetVariableValue.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pnp_msgs
)
_generate_srv_eus(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPCondition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pnp_msgs
)
_generate_srv_eus(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPReward.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pnp_msgs
)
_generate_srv_eus(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPGetVariableValue.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pnp_msgs
)
_generate_srv_eus(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPLastEvent.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pnp_msgs
)
_generate_srv_eus(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPClearBuffer.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pnp_msgs
)

### Generating Module File
_generate_module_eus(pnp_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pnp_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(pnp_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(pnp_msgs_generate_messages pnp_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPLastEvent.srv" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_eus _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPSetVariableValue.srv" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_eus _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPCondition.srv" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_eus _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionResult.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_eus _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPAction.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_eus _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionGoal.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_eus _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/msg/Action.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_eus _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPGetVariableValue.srv" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_eus _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPFeedback.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_eus _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionFeedback.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_eus _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPGoal.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_eus _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/msg/ActionFinished.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_eus _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPResult.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_eus _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPReward.srv" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_eus _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPClearBuffer.srv" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_eus _pnp_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pnp_msgs_geneus)
add_dependencies(pnp_msgs_geneus pnp_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pnp_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pnp_msgs
)
_generate_msg_lisp(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionResult.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPGoal.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPFeedback.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pnp_msgs
)
_generate_msg_lisp(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pnp_msgs
)
_generate_msg_lisp(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/msg/Action.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pnp_msgs
)
_generate_msg_lisp(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pnp_msgs
)
_generate_msg_lisp(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pnp_msgs
)
_generate_msg_lisp(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pnp_msgs
)
_generate_msg_lisp(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/msg/ActionFinished.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pnp_msgs
)
_generate_msg_lisp(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pnp_msgs
)

### Generating Services
_generate_srv_lisp(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPSetVariableValue.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pnp_msgs
)
_generate_srv_lisp(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPCondition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pnp_msgs
)
_generate_srv_lisp(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPReward.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pnp_msgs
)
_generate_srv_lisp(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPGetVariableValue.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pnp_msgs
)
_generate_srv_lisp(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPLastEvent.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pnp_msgs
)
_generate_srv_lisp(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPClearBuffer.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pnp_msgs
)

### Generating Module File
_generate_module_lisp(pnp_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pnp_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(pnp_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(pnp_msgs_generate_messages pnp_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPLastEvent.srv" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_lisp _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPSetVariableValue.srv" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_lisp _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPCondition.srv" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_lisp _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionResult.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_lisp _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPAction.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_lisp _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionGoal.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_lisp _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/msg/Action.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_lisp _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPGetVariableValue.srv" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_lisp _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPFeedback.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_lisp _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionFeedback.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_lisp _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPGoal.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_lisp _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/msg/ActionFinished.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_lisp _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPResult.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_lisp _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPReward.srv" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_lisp _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPClearBuffer.srv" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_lisp _pnp_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pnp_msgs_genlisp)
add_dependencies(pnp_msgs_genlisp pnp_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pnp_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pnp_msgs
)
_generate_msg_nodejs(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionResult.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPGoal.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPFeedback.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pnp_msgs
)
_generate_msg_nodejs(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pnp_msgs
)
_generate_msg_nodejs(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/msg/Action.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pnp_msgs
)
_generate_msg_nodejs(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pnp_msgs
)
_generate_msg_nodejs(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pnp_msgs
)
_generate_msg_nodejs(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pnp_msgs
)
_generate_msg_nodejs(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/msg/ActionFinished.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pnp_msgs
)
_generate_msg_nodejs(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pnp_msgs
)

### Generating Services
_generate_srv_nodejs(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPSetVariableValue.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pnp_msgs
)
_generate_srv_nodejs(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPCondition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pnp_msgs
)
_generate_srv_nodejs(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPReward.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pnp_msgs
)
_generate_srv_nodejs(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPGetVariableValue.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pnp_msgs
)
_generate_srv_nodejs(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPLastEvent.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pnp_msgs
)
_generate_srv_nodejs(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPClearBuffer.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pnp_msgs
)

### Generating Module File
_generate_module_nodejs(pnp_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pnp_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(pnp_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(pnp_msgs_generate_messages pnp_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPLastEvent.srv" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_nodejs _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPSetVariableValue.srv" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_nodejs _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPCondition.srv" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_nodejs _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionResult.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_nodejs _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPAction.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_nodejs _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionGoal.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_nodejs _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/msg/Action.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_nodejs _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPGetVariableValue.srv" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_nodejs _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPFeedback.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_nodejs _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionFeedback.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_nodejs _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPGoal.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_nodejs _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/msg/ActionFinished.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_nodejs _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPResult.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_nodejs _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPReward.srv" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_nodejs _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPClearBuffer.srv" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_nodejs _pnp_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pnp_msgs_gennodejs)
add_dependencies(pnp_msgs_gennodejs pnp_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pnp_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pnp_msgs
)
_generate_msg_py(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionResult.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPGoal.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPFeedback.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pnp_msgs
)
_generate_msg_py(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pnp_msgs
)
_generate_msg_py(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/msg/Action.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pnp_msgs
)
_generate_msg_py(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pnp_msgs
)
_generate_msg_py(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pnp_msgs
)
_generate_msg_py(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pnp_msgs
)
_generate_msg_py(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/msg/ActionFinished.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pnp_msgs
)
_generate_msg_py(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pnp_msgs
)

### Generating Services
_generate_srv_py(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPSetVariableValue.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pnp_msgs
)
_generate_srv_py(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPCondition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pnp_msgs
)
_generate_srv_py(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPReward.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pnp_msgs
)
_generate_srv_py(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPGetVariableValue.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pnp_msgs
)
_generate_srv_py(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPLastEvent.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pnp_msgs
)
_generate_srv_py(pnp_msgs
  "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPClearBuffer.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pnp_msgs
)

### Generating Module File
_generate_module_py(pnp_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pnp_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(pnp_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(pnp_msgs_generate_messages pnp_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPLastEvent.srv" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_py _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPSetVariableValue.srv" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_py _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPCondition.srv" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_py _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionResult.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_py _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPAction.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_py _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionGoal.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_py _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/msg/Action.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_py _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPGetVariableValue.srv" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_py _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPFeedback.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_py _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPActionFeedback.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_py _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPGoal.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_py _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/msg/ActionFinished.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_py _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/cmake-build-debug/devel/share/pnp_msgs/msg/PNPResult.msg" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_py _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPReward.srv" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_py _pnp_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/pnp_msgs/srv/PNPClearBuffer.srv" NAME_WE)
add_dependencies(pnp_msgs_generate_messages_py _pnp_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pnp_msgs_genpy)
add_dependencies(pnp_msgs_genpy pnp_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pnp_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pnp_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pnp_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(pnp_msgs_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pnp_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pnp_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(pnp_msgs_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pnp_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pnp_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(pnp_msgs_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pnp_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pnp_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(pnp_msgs_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pnp_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pnp_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pnp_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(pnp_msgs_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
