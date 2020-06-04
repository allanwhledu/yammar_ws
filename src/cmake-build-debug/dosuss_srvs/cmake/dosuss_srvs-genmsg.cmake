# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "dosuss_srvs: 1 messages, 1 services")

set(MSG_I_FLAGS "-Idosuss_srvs:/home/sjtu_wanghaili/yammar_ws/src/dosuss_srvs/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(dosuss_srvs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/dosuss_srvs/srv/find_cut.srv" NAME_WE)
add_custom_target(_dosuss_srvs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dosuss_srvs" "/home/sjtu_wanghaili/yammar_ws/src/dosuss_srvs/srv/find_cut.srv" ""
)

get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/dosuss_srvs/msg/border_msg.msg" NAME_WE)
add_custom_target(_dosuss_srvs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dosuss_srvs" "/home/sjtu_wanghaili/yammar_ws/src/dosuss_srvs/msg/border_msg.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(dosuss_srvs
  "/home/sjtu_wanghaili/yammar_ws/src/dosuss_srvs/msg/border_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dosuss_srvs
)

### Generating Services
_generate_srv_cpp(dosuss_srvs
  "/home/sjtu_wanghaili/yammar_ws/src/dosuss_srvs/srv/find_cut.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dosuss_srvs
)

### Generating Module File
_generate_module_cpp(dosuss_srvs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dosuss_srvs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(dosuss_srvs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(dosuss_srvs_generate_messages dosuss_srvs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/dosuss_srvs/srv/find_cut.srv" NAME_WE)
add_dependencies(dosuss_srvs_generate_messages_cpp _dosuss_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/dosuss_srvs/msg/border_msg.msg" NAME_WE)
add_dependencies(dosuss_srvs_generate_messages_cpp _dosuss_srvs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dosuss_srvs_gencpp)
add_dependencies(dosuss_srvs_gencpp dosuss_srvs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dosuss_srvs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(dosuss_srvs
  "/home/sjtu_wanghaili/yammar_ws/src/dosuss_srvs/msg/border_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dosuss_srvs
)

### Generating Services
_generate_srv_eus(dosuss_srvs
  "/home/sjtu_wanghaili/yammar_ws/src/dosuss_srvs/srv/find_cut.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dosuss_srvs
)

### Generating Module File
_generate_module_eus(dosuss_srvs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dosuss_srvs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(dosuss_srvs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(dosuss_srvs_generate_messages dosuss_srvs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/dosuss_srvs/srv/find_cut.srv" NAME_WE)
add_dependencies(dosuss_srvs_generate_messages_eus _dosuss_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/dosuss_srvs/msg/border_msg.msg" NAME_WE)
add_dependencies(dosuss_srvs_generate_messages_eus _dosuss_srvs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dosuss_srvs_geneus)
add_dependencies(dosuss_srvs_geneus dosuss_srvs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dosuss_srvs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(dosuss_srvs
  "/home/sjtu_wanghaili/yammar_ws/src/dosuss_srvs/msg/border_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dosuss_srvs
)

### Generating Services
_generate_srv_lisp(dosuss_srvs
  "/home/sjtu_wanghaili/yammar_ws/src/dosuss_srvs/srv/find_cut.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dosuss_srvs
)

### Generating Module File
_generate_module_lisp(dosuss_srvs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dosuss_srvs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(dosuss_srvs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(dosuss_srvs_generate_messages dosuss_srvs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/dosuss_srvs/srv/find_cut.srv" NAME_WE)
add_dependencies(dosuss_srvs_generate_messages_lisp _dosuss_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/dosuss_srvs/msg/border_msg.msg" NAME_WE)
add_dependencies(dosuss_srvs_generate_messages_lisp _dosuss_srvs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dosuss_srvs_genlisp)
add_dependencies(dosuss_srvs_genlisp dosuss_srvs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dosuss_srvs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(dosuss_srvs
  "/home/sjtu_wanghaili/yammar_ws/src/dosuss_srvs/msg/border_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dosuss_srvs
)

### Generating Services
_generate_srv_nodejs(dosuss_srvs
  "/home/sjtu_wanghaili/yammar_ws/src/dosuss_srvs/srv/find_cut.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dosuss_srvs
)

### Generating Module File
_generate_module_nodejs(dosuss_srvs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dosuss_srvs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(dosuss_srvs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(dosuss_srvs_generate_messages dosuss_srvs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/dosuss_srvs/srv/find_cut.srv" NAME_WE)
add_dependencies(dosuss_srvs_generate_messages_nodejs _dosuss_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/dosuss_srvs/msg/border_msg.msg" NAME_WE)
add_dependencies(dosuss_srvs_generate_messages_nodejs _dosuss_srvs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dosuss_srvs_gennodejs)
add_dependencies(dosuss_srvs_gennodejs dosuss_srvs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dosuss_srvs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(dosuss_srvs
  "/home/sjtu_wanghaili/yammar_ws/src/dosuss_srvs/msg/border_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dosuss_srvs
)

### Generating Services
_generate_srv_py(dosuss_srvs
  "/home/sjtu_wanghaili/yammar_ws/src/dosuss_srvs/srv/find_cut.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dosuss_srvs
)

### Generating Module File
_generate_module_py(dosuss_srvs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dosuss_srvs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(dosuss_srvs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(dosuss_srvs_generate_messages dosuss_srvs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/dosuss_srvs/srv/find_cut.srv" NAME_WE)
add_dependencies(dosuss_srvs_generate_messages_py _dosuss_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sjtu_wanghaili/yammar_ws/src/dosuss_srvs/msg/border_msg.msg" NAME_WE)
add_dependencies(dosuss_srvs_generate_messages_py _dosuss_srvs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dosuss_srvs_genpy)
add_dependencies(dosuss_srvs_genpy dosuss_srvs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dosuss_srvs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dosuss_srvs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dosuss_srvs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(dosuss_srvs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dosuss_srvs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dosuss_srvs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(dosuss_srvs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dosuss_srvs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dosuss_srvs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(dosuss_srvs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dosuss_srvs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dosuss_srvs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(dosuss_srvs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dosuss_srvs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dosuss_srvs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dosuss_srvs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(dosuss_srvs_generate_messages_py std_msgs_generate_messages_py)
endif()
