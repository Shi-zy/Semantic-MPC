# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "semantics_mpc: 3 messages, 2 services")

set(MSG_I_FLAGS "-Isemantics_mpc:/root/new/catkin_ws/src/Semantics-MPC/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(semantics_mpc_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleInfo.msg" NAME_WE)
add_custom_target(_semantics_mpc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "semantics_mpc" "/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleInfo.msg" "std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Point:geometry_msgs/Vector3"
)

get_filename_component(_filename "/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleArray.msg" NAME_WE)
add_custom_target(_semantics_mpc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "semantics_mpc" "/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleArray.msg" "geometry_msgs/Quaternion:std_msgs/Header:semantics_mpc/ObstacleInfo:geometry_msgs/Point:geometry_msgs/Vector3"
)

get_filename_component(_filename "/root/new/catkin_ws/src/Semantics-MPC/msg/DetectionResult.msg" NAME_WE)
add_custom_target(_semantics_mpc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "semantics_mpc" "/root/new/catkin_ws/src/Semantics-MPC/msg/DetectionResult.msg" "geometry_msgs/Quaternion:std_msgs/Header:semantics_mpc/ObstacleInfo:geometry_msgs/Point:geometry_msgs/Vector3:semantics_mpc/ObstacleArray"
)

get_filename_component(_filename "/root/new/catkin_ws/src/Semantics-MPC/srv/GetStaticObstacles.srv" NAME_WE)
add_custom_target(_semantics_mpc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "semantics_mpc" "/root/new/catkin_ws/src/Semantics-MPC/srv/GetStaticObstacles.srv" "geometry_msgs/Quaternion:std_msgs/Header:semantics_mpc/ObstacleInfo:geometry_msgs/Point:geometry_msgs/Vector3:semantics_mpc/ObstacleArray"
)

get_filename_component(_filename "/root/new/catkin_ws/src/Semantics-MPC/srv/GetDynamicObstacles.srv" NAME_WE)
add_custom_target(_semantics_mpc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "semantics_mpc" "/root/new/catkin_ws/src/Semantics-MPC/srv/GetDynamicObstacles.srv" "geometry_msgs/Quaternion:std_msgs/Header:semantics_mpc/ObstacleInfo:geometry_msgs/Point:geometry_msgs/Vector3:semantics_mpc/ObstacleArray"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(semantics_mpc
  "/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/semantics_mpc
)
_generate_msg_cpp(semantics_mpc
  "/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleInfo.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/semantics_mpc
)
_generate_msg_cpp(semantics_mpc
  "/root/new/catkin_ws/src/Semantics-MPC/msg/DetectionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleInfo.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/semantics_mpc
)

### Generating Services
_generate_srv_cpp(semantics_mpc
  "/root/new/catkin_ws/src/Semantics-MPC/srv/GetStaticObstacles.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleInfo.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/semantics_mpc
)
_generate_srv_cpp(semantics_mpc
  "/root/new/catkin_ws/src/Semantics-MPC/srv/GetDynamicObstacles.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleInfo.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/semantics_mpc
)

### Generating Module File
_generate_module_cpp(semantics_mpc
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/semantics_mpc
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(semantics_mpc_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(semantics_mpc_generate_messages semantics_mpc_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleInfo.msg" NAME_WE)
add_dependencies(semantics_mpc_generate_messages_cpp _semantics_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleArray.msg" NAME_WE)
add_dependencies(semantics_mpc_generate_messages_cpp _semantics_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/new/catkin_ws/src/Semantics-MPC/msg/DetectionResult.msg" NAME_WE)
add_dependencies(semantics_mpc_generate_messages_cpp _semantics_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/new/catkin_ws/src/Semantics-MPC/srv/GetStaticObstacles.srv" NAME_WE)
add_dependencies(semantics_mpc_generate_messages_cpp _semantics_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/new/catkin_ws/src/Semantics-MPC/srv/GetDynamicObstacles.srv" NAME_WE)
add_dependencies(semantics_mpc_generate_messages_cpp _semantics_mpc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(semantics_mpc_gencpp)
add_dependencies(semantics_mpc_gencpp semantics_mpc_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS semantics_mpc_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(semantics_mpc
  "/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/semantics_mpc
)
_generate_msg_eus(semantics_mpc
  "/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleInfo.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/semantics_mpc
)
_generate_msg_eus(semantics_mpc
  "/root/new/catkin_ws/src/Semantics-MPC/msg/DetectionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleInfo.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/semantics_mpc
)

### Generating Services
_generate_srv_eus(semantics_mpc
  "/root/new/catkin_ws/src/Semantics-MPC/srv/GetStaticObstacles.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleInfo.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/semantics_mpc
)
_generate_srv_eus(semantics_mpc
  "/root/new/catkin_ws/src/Semantics-MPC/srv/GetDynamicObstacles.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleInfo.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/semantics_mpc
)

### Generating Module File
_generate_module_eus(semantics_mpc
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/semantics_mpc
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(semantics_mpc_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(semantics_mpc_generate_messages semantics_mpc_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleInfo.msg" NAME_WE)
add_dependencies(semantics_mpc_generate_messages_eus _semantics_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleArray.msg" NAME_WE)
add_dependencies(semantics_mpc_generate_messages_eus _semantics_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/new/catkin_ws/src/Semantics-MPC/msg/DetectionResult.msg" NAME_WE)
add_dependencies(semantics_mpc_generate_messages_eus _semantics_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/new/catkin_ws/src/Semantics-MPC/srv/GetStaticObstacles.srv" NAME_WE)
add_dependencies(semantics_mpc_generate_messages_eus _semantics_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/new/catkin_ws/src/Semantics-MPC/srv/GetDynamicObstacles.srv" NAME_WE)
add_dependencies(semantics_mpc_generate_messages_eus _semantics_mpc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(semantics_mpc_geneus)
add_dependencies(semantics_mpc_geneus semantics_mpc_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS semantics_mpc_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(semantics_mpc
  "/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/semantics_mpc
)
_generate_msg_lisp(semantics_mpc
  "/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleInfo.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/semantics_mpc
)
_generate_msg_lisp(semantics_mpc
  "/root/new/catkin_ws/src/Semantics-MPC/msg/DetectionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleInfo.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/semantics_mpc
)

### Generating Services
_generate_srv_lisp(semantics_mpc
  "/root/new/catkin_ws/src/Semantics-MPC/srv/GetStaticObstacles.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleInfo.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/semantics_mpc
)
_generate_srv_lisp(semantics_mpc
  "/root/new/catkin_ws/src/Semantics-MPC/srv/GetDynamicObstacles.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleInfo.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/semantics_mpc
)

### Generating Module File
_generate_module_lisp(semantics_mpc
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/semantics_mpc
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(semantics_mpc_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(semantics_mpc_generate_messages semantics_mpc_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleInfo.msg" NAME_WE)
add_dependencies(semantics_mpc_generate_messages_lisp _semantics_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleArray.msg" NAME_WE)
add_dependencies(semantics_mpc_generate_messages_lisp _semantics_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/new/catkin_ws/src/Semantics-MPC/msg/DetectionResult.msg" NAME_WE)
add_dependencies(semantics_mpc_generate_messages_lisp _semantics_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/new/catkin_ws/src/Semantics-MPC/srv/GetStaticObstacles.srv" NAME_WE)
add_dependencies(semantics_mpc_generate_messages_lisp _semantics_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/new/catkin_ws/src/Semantics-MPC/srv/GetDynamicObstacles.srv" NAME_WE)
add_dependencies(semantics_mpc_generate_messages_lisp _semantics_mpc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(semantics_mpc_genlisp)
add_dependencies(semantics_mpc_genlisp semantics_mpc_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS semantics_mpc_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(semantics_mpc
  "/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/semantics_mpc
)
_generate_msg_nodejs(semantics_mpc
  "/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleInfo.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/semantics_mpc
)
_generate_msg_nodejs(semantics_mpc
  "/root/new/catkin_ws/src/Semantics-MPC/msg/DetectionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleInfo.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/semantics_mpc
)

### Generating Services
_generate_srv_nodejs(semantics_mpc
  "/root/new/catkin_ws/src/Semantics-MPC/srv/GetStaticObstacles.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleInfo.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/semantics_mpc
)
_generate_srv_nodejs(semantics_mpc
  "/root/new/catkin_ws/src/Semantics-MPC/srv/GetDynamicObstacles.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleInfo.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/semantics_mpc
)

### Generating Module File
_generate_module_nodejs(semantics_mpc
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/semantics_mpc
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(semantics_mpc_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(semantics_mpc_generate_messages semantics_mpc_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleInfo.msg" NAME_WE)
add_dependencies(semantics_mpc_generate_messages_nodejs _semantics_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleArray.msg" NAME_WE)
add_dependencies(semantics_mpc_generate_messages_nodejs _semantics_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/new/catkin_ws/src/Semantics-MPC/msg/DetectionResult.msg" NAME_WE)
add_dependencies(semantics_mpc_generate_messages_nodejs _semantics_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/new/catkin_ws/src/Semantics-MPC/srv/GetStaticObstacles.srv" NAME_WE)
add_dependencies(semantics_mpc_generate_messages_nodejs _semantics_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/new/catkin_ws/src/Semantics-MPC/srv/GetDynamicObstacles.srv" NAME_WE)
add_dependencies(semantics_mpc_generate_messages_nodejs _semantics_mpc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(semantics_mpc_gennodejs)
add_dependencies(semantics_mpc_gennodejs semantics_mpc_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS semantics_mpc_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(semantics_mpc
  "/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/semantics_mpc
)
_generate_msg_py(semantics_mpc
  "/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleInfo.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/semantics_mpc
)
_generate_msg_py(semantics_mpc
  "/root/new/catkin_ws/src/Semantics-MPC/msg/DetectionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleInfo.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/semantics_mpc
)

### Generating Services
_generate_srv_py(semantics_mpc
  "/root/new/catkin_ws/src/Semantics-MPC/srv/GetStaticObstacles.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleInfo.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/semantics_mpc
)
_generate_srv_py(semantics_mpc
  "/root/new/catkin_ws/src/Semantics-MPC/srv/GetDynamicObstacles.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleInfo.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/semantics_mpc
)

### Generating Module File
_generate_module_py(semantics_mpc
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/semantics_mpc
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(semantics_mpc_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(semantics_mpc_generate_messages semantics_mpc_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleInfo.msg" NAME_WE)
add_dependencies(semantics_mpc_generate_messages_py _semantics_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/new/catkin_ws/src/Semantics-MPC/msg/ObstacleArray.msg" NAME_WE)
add_dependencies(semantics_mpc_generate_messages_py _semantics_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/new/catkin_ws/src/Semantics-MPC/msg/DetectionResult.msg" NAME_WE)
add_dependencies(semantics_mpc_generate_messages_py _semantics_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/new/catkin_ws/src/Semantics-MPC/srv/GetStaticObstacles.srv" NAME_WE)
add_dependencies(semantics_mpc_generate_messages_py _semantics_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/new/catkin_ws/src/Semantics-MPC/srv/GetDynamicObstacles.srv" NAME_WE)
add_dependencies(semantics_mpc_generate_messages_py _semantics_mpc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(semantics_mpc_genpy)
add_dependencies(semantics_mpc_genpy semantics_mpc_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS semantics_mpc_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/semantics_mpc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/semantics_mpc
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(semantics_mpc_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(semantics_mpc_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/semantics_mpc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/semantics_mpc
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(semantics_mpc_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(semantics_mpc_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/semantics_mpc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/semantics_mpc
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(semantics_mpc_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(semantics_mpc_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/semantics_mpc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/semantics_mpc
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(semantics_mpc_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(semantics_mpc_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/semantics_mpc)
  install(CODE "execute_process(COMMAND \"/root/miniconda3/envs/myconda/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/semantics_mpc\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/semantics_mpc
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(semantics_mpc_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(semantics_mpc_generate_messages_py geometry_msgs_generate_messages_py)
endif()
