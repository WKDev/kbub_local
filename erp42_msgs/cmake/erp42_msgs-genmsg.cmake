# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "erp42_msgs: 5 messages, 0 services")

set(MSG_I_FLAGS "-Ierp42_msgs:/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(erp42_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/CmdControl.msg" NAME_WE)
add_custom_target(_erp42_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "erp42_msgs" "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/CmdControl.msg" ""
)

get_filename_component(_filename "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/DriveCmd.msg" NAME_WE)
add_custom_target(_erp42_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "erp42_msgs" "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/DriveCmd.msg" ""
)

get_filename_component(_filename "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/ModeCmd.msg" NAME_WE)
add_custom_target(_erp42_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "erp42_msgs" "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/ModeCmd.msg" ""
)

get_filename_component(_filename "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/CANFeedBack.msg" NAME_WE)
add_custom_target(_erp42_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "erp42_msgs" "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/CANFeedBack.msg" ""
)

get_filename_component(_filename "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/SerialFeedBack.msg" NAME_WE)
add_custom_target(_erp42_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "erp42_msgs" "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/SerialFeedBack.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(erp42_msgs
  "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/CmdControl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/erp42_msgs
)
_generate_msg_cpp(erp42_msgs
  "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/DriveCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/erp42_msgs
)
_generate_msg_cpp(erp42_msgs
  "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/ModeCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/erp42_msgs
)
_generate_msg_cpp(erp42_msgs
  "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/CANFeedBack.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/erp42_msgs
)
_generate_msg_cpp(erp42_msgs
  "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/SerialFeedBack.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/erp42_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(erp42_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/erp42_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(erp42_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(erp42_msgs_generate_messages erp42_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/CmdControl.msg" NAME_WE)
add_dependencies(erp42_msgs_generate_messages_cpp _erp42_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/DriveCmd.msg" NAME_WE)
add_dependencies(erp42_msgs_generate_messages_cpp _erp42_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/ModeCmd.msg" NAME_WE)
add_dependencies(erp42_msgs_generate_messages_cpp _erp42_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/CANFeedBack.msg" NAME_WE)
add_dependencies(erp42_msgs_generate_messages_cpp _erp42_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/SerialFeedBack.msg" NAME_WE)
add_dependencies(erp42_msgs_generate_messages_cpp _erp42_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(erp42_msgs_gencpp)
add_dependencies(erp42_msgs_gencpp erp42_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS erp42_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(erp42_msgs
  "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/CmdControl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/erp42_msgs
)
_generate_msg_eus(erp42_msgs
  "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/DriveCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/erp42_msgs
)
_generate_msg_eus(erp42_msgs
  "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/ModeCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/erp42_msgs
)
_generate_msg_eus(erp42_msgs
  "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/CANFeedBack.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/erp42_msgs
)
_generate_msg_eus(erp42_msgs
  "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/SerialFeedBack.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/erp42_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(erp42_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/erp42_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(erp42_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(erp42_msgs_generate_messages erp42_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/CmdControl.msg" NAME_WE)
add_dependencies(erp42_msgs_generate_messages_eus _erp42_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/DriveCmd.msg" NAME_WE)
add_dependencies(erp42_msgs_generate_messages_eus _erp42_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/ModeCmd.msg" NAME_WE)
add_dependencies(erp42_msgs_generate_messages_eus _erp42_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/CANFeedBack.msg" NAME_WE)
add_dependencies(erp42_msgs_generate_messages_eus _erp42_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/SerialFeedBack.msg" NAME_WE)
add_dependencies(erp42_msgs_generate_messages_eus _erp42_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(erp42_msgs_geneus)
add_dependencies(erp42_msgs_geneus erp42_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS erp42_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(erp42_msgs
  "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/CmdControl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/erp42_msgs
)
_generate_msg_lisp(erp42_msgs
  "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/DriveCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/erp42_msgs
)
_generate_msg_lisp(erp42_msgs
  "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/ModeCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/erp42_msgs
)
_generate_msg_lisp(erp42_msgs
  "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/CANFeedBack.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/erp42_msgs
)
_generate_msg_lisp(erp42_msgs
  "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/SerialFeedBack.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/erp42_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(erp42_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/erp42_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(erp42_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(erp42_msgs_generate_messages erp42_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/CmdControl.msg" NAME_WE)
add_dependencies(erp42_msgs_generate_messages_lisp _erp42_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/DriveCmd.msg" NAME_WE)
add_dependencies(erp42_msgs_generate_messages_lisp _erp42_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/ModeCmd.msg" NAME_WE)
add_dependencies(erp42_msgs_generate_messages_lisp _erp42_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/CANFeedBack.msg" NAME_WE)
add_dependencies(erp42_msgs_generate_messages_lisp _erp42_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/SerialFeedBack.msg" NAME_WE)
add_dependencies(erp42_msgs_generate_messages_lisp _erp42_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(erp42_msgs_genlisp)
add_dependencies(erp42_msgs_genlisp erp42_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS erp42_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(erp42_msgs
  "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/CmdControl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/erp42_msgs
)
_generate_msg_nodejs(erp42_msgs
  "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/DriveCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/erp42_msgs
)
_generate_msg_nodejs(erp42_msgs
  "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/ModeCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/erp42_msgs
)
_generate_msg_nodejs(erp42_msgs
  "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/CANFeedBack.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/erp42_msgs
)
_generate_msg_nodejs(erp42_msgs
  "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/SerialFeedBack.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/erp42_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(erp42_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/erp42_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(erp42_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(erp42_msgs_generate_messages erp42_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/CmdControl.msg" NAME_WE)
add_dependencies(erp42_msgs_generate_messages_nodejs _erp42_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/DriveCmd.msg" NAME_WE)
add_dependencies(erp42_msgs_generate_messages_nodejs _erp42_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/ModeCmd.msg" NAME_WE)
add_dependencies(erp42_msgs_generate_messages_nodejs _erp42_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/CANFeedBack.msg" NAME_WE)
add_dependencies(erp42_msgs_generate_messages_nodejs _erp42_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/SerialFeedBack.msg" NAME_WE)
add_dependencies(erp42_msgs_generate_messages_nodejs _erp42_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(erp42_msgs_gennodejs)
add_dependencies(erp42_msgs_gennodejs erp42_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS erp42_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(erp42_msgs
  "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/CmdControl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/erp42_msgs
)
_generate_msg_py(erp42_msgs
  "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/DriveCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/erp42_msgs
)
_generate_msg_py(erp42_msgs
  "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/ModeCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/erp42_msgs
)
_generate_msg_py(erp42_msgs
  "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/CANFeedBack.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/erp42_msgs
)
_generate_msg_py(erp42_msgs
  "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/SerialFeedBack.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/erp42_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(erp42_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/erp42_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(erp42_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(erp42_msgs_generate_messages erp42_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/CmdControl.msg" NAME_WE)
add_dependencies(erp42_msgs_generate_messages_py _erp42_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/DriveCmd.msg" NAME_WE)
add_dependencies(erp42_msgs_generate_messages_py _erp42_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/ModeCmd.msg" NAME_WE)
add_dependencies(erp42_msgs_generate_messages_py _erp42_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/CANFeedBack.msg" NAME_WE)
add_dependencies(erp42_msgs_generate_messages_py _erp42_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/parallels/erp42_melodic_ws/src/ERP42-ROS/packages/erp42_msgs/msg/SerialFeedBack.msg" NAME_WE)
add_dependencies(erp42_msgs_generate_messages_py _erp42_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(erp42_msgs_genpy)
add_dependencies(erp42_msgs_genpy erp42_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS erp42_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/erp42_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/erp42_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(erp42_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/erp42_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/erp42_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(erp42_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/erp42_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/erp42_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(erp42_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/erp42_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/erp42_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(erp42_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/erp42_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/erp42_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/erp42_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(erp42_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
