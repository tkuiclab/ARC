# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "robotis_controller_msgs: 3 messages, 1 services")

set(MSG_I_FLAGS "-Irobotis_controller_msgs:/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(robotis_controller_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg/SyncWriteItem.msg" NAME_WE)
add_custom_target(_robotis_controller_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robotis_controller_msgs" "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg/SyncWriteItem.msg" ""
)

get_filename_component(_filename "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg/StatusMsg.msg" NAME_WE)
add_custom_target(_robotis_controller_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robotis_controller_msgs" "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg/StatusMsg.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/srv/GetJointModule.srv" NAME_WE)
add_custom_target(_robotis_controller_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robotis_controller_msgs" "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/srv/GetJointModule.srv" ""
)

get_filename_component(_filename "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg/JointCtrlModule.msg" NAME_WE)
add_custom_target(_robotis_controller_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robotis_controller_msgs" "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg/JointCtrlModule.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(robotis_controller_msgs
  "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg/SyncWriteItem.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotis_controller_msgs
)
_generate_msg_cpp(robotis_controller_msgs
  "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg/StatusMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotis_controller_msgs
)
_generate_msg_cpp(robotis_controller_msgs
  "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg/JointCtrlModule.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotis_controller_msgs
)

### Generating Services
_generate_srv_cpp(robotis_controller_msgs
  "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/srv/GetJointModule.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotis_controller_msgs
)

### Generating Module File
_generate_module_cpp(robotis_controller_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotis_controller_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(robotis_controller_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(robotis_controller_msgs_generate_messages robotis_controller_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg/SyncWriteItem.msg" NAME_WE)
add_dependencies(robotis_controller_msgs_generate_messages_cpp _robotis_controller_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg/StatusMsg.msg" NAME_WE)
add_dependencies(robotis_controller_msgs_generate_messages_cpp _robotis_controller_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/srv/GetJointModule.srv" NAME_WE)
add_dependencies(robotis_controller_msgs_generate_messages_cpp _robotis_controller_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg/JointCtrlModule.msg" NAME_WE)
add_dependencies(robotis_controller_msgs_generate_messages_cpp _robotis_controller_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotis_controller_msgs_gencpp)
add_dependencies(robotis_controller_msgs_gencpp robotis_controller_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotis_controller_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(robotis_controller_msgs
  "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg/SyncWriteItem.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotis_controller_msgs
)
_generate_msg_eus(robotis_controller_msgs
  "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg/StatusMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotis_controller_msgs
)
_generate_msg_eus(robotis_controller_msgs
  "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg/JointCtrlModule.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotis_controller_msgs
)

### Generating Services
_generate_srv_eus(robotis_controller_msgs
  "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/srv/GetJointModule.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotis_controller_msgs
)

### Generating Module File
_generate_module_eus(robotis_controller_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotis_controller_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(robotis_controller_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(robotis_controller_msgs_generate_messages robotis_controller_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg/SyncWriteItem.msg" NAME_WE)
add_dependencies(robotis_controller_msgs_generate_messages_eus _robotis_controller_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg/StatusMsg.msg" NAME_WE)
add_dependencies(robotis_controller_msgs_generate_messages_eus _robotis_controller_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/srv/GetJointModule.srv" NAME_WE)
add_dependencies(robotis_controller_msgs_generate_messages_eus _robotis_controller_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg/JointCtrlModule.msg" NAME_WE)
add_dependencies(robotis_controller_msgs_generate_messages_eus _robotis_controller_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotis_controller_msgs_geneus)
add_dependencies(robotis_controller_msgs_geneus robotis_controller_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotis_controller_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(robotis_controller_msgs
  "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg/SyncWriteItem.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotis_controller_msgs
)
_generate_msg_lisp(robotis_controller_msgs
  "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg/StatusMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotis_controller_msgs
)
_generate_msg_lisp(robotis_controller_msgs
  "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg/JointCtrlModule.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotis_controller_msgs
)

### Generating Services
_generate_srv_lisp(robotis_controller_msgs
  "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/srv/GetJointModule.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotis_controller_msgs
)

### Generating Module File
_generate_module_lisp(robotis_controller_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotis_controller_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(robotis_controller_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(robotis_controller_msgs_generate_messages robotis_controller_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg/SyncWriteItem.msg" NAME_WE)
add_dependencies(robotis_controller_msgs_generate_messages_lisp _robotis_controller_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg/StatusMsg.msg" NAME_WE)
add_dependencies(robotis_controller_msgs_generate_messages_lisp _robotis_controller_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/srv/GetJointModule.srv" NAME_WE)
add_dependencies(robotis_controller_msgs_generate_messages_lisp _robotis_controller_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg/JointCtrlModule.msg" NAME_WE)
add_dependencies(robotis_controller_msgs_generate_messages_lisp _robotis_controller_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotis_controller_msgs_genlisp)
add_dependencies(robotis_controller_msgs_genlisp robotis_controller_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotis_controller_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(robotis_controller_msgs
  "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg/SyncWriteItem.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotis_controller_msgs
)
_generate_msg_nodejs(robotis_controller_msgs
  "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg/StatusMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotis_controller_msgs
)
_generate_msg_nodejs(robotis_controller_msgs
  "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg/JointCtrlModule.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotis_controller_msgs
)

### Generating Services
_generate_srv_nodejs(robotis_controller_msgs
  "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/srv/GetJointModule.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotis_controller_msgs
)

### Generating Module File
_generate_module_nodejs(robotis_controller_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotis_controller_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(robotis_controller_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(robotis_controller_msgs_generate_messages robotis_controller_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg/SyncWriteItem.msg" NAME_WE)
add_dependencies(robotis_controller_msgs_generate_messages_nodejs _robotis_controller_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg/StatusMsg.msg" NAME_WE)
add_dependencies(robotis_controller_msgs_generate_messages_nodejs _robotis_controller_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/srv/GetJointModule.srv" NAME_WE)
add_dependencies(robotis_controller_msgs_generate_messages_nodejs _robotis_controller_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg/JointCtrlModule.msg" NAME_WE)
add_dependencies(robotis_controller_msgs_generate_messages_nodejs _robotis_controller_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotis_controller_msgs_gennodejs)
add_dependencies(robotis_controller_msgs_gennodejs robotis_controller_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotis_controller_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(robotis_controller_msgs
  "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg/SyncWriteItem.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotis_controller_msgs
)
_generate_msg_py(robotis_controller_msgs
  "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg/StatusMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotis_controller_msgs
)
_generate_msg_py(robotis_controller_msgs
  "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg/JointCtrlModule.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotis_controller_msgs
)

### Generating Services
_generate_srv_py(robotis_controller_msgs
  "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/srv/GetJointModule.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotis_controller_msgs
)

### Generating Module File
_generate_module_py(robotis_controller_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotis_controller_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(robotis_controller_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(robotis_controller_msgs_generate_messages robotis_controller_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg/SyncWriteItem.msg" NAME_WE)
add_dependencies(robotis_controller_msgs_generate_messages_py _robotis_controller_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg/StatusMsg.msg" NAME_WE)
add_dependencies(robotis_controller_msgs_generate_messages_py _robotis_controller_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/srv/GetJointModule.srv" NAME_WE)
add_dependencies(robotis_controller_msgs_generate_messages_py _robotis_controller_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/iclab/arc_ws/src/manipulator_7a/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg/JointCtrlModule.msg" NAME_WE)
add_dependencies(robotis_controller_msgs_generate_messages_py _robotis_controller_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotis_controller_msgs_genpy)
add_dependencies(robotis_controller_msgs_genpy robotis_controller_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotis_controller_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotis_controller_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotis_controller_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(robotis_controller_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(robotis_controller_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotis_controller_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotis_controller_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(robotis_controller_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(robotis_controller_msgs_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotis_controller_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotis_controller_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(robotis_controller_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(robotis_controller_msgs_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotis_controller_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotis_controller_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(robotis_controller_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(robotis_controller_msgs_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotis_controller_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotis_controller_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotis_controller_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(robotis_controller_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(robotis_controller_msgs_generate_messages_py sensor_msgs_generate_messages_py)
endif()
