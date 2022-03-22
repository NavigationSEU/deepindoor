# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "object_tracking: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iobject_tracking:/home/wang/2.24/object_tracking2/src/object_tracking/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Ivisualization_msgs:/opt/ros/melodic/share/visualization_msgs/cmake/../msg;-Ijsk_recognition_msgs:/opt/ros/melodic/share/jsk_recognition_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/melodic/share/sensor_msgs/cmake/../msg;-Ipcl_msgs:/opt/ros/melodic/share/pcl_msgs/cmake/../msg;-Ijsk_footstep_msgs:/opt/ros/melodic/share/jsk_footstep_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/melodic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(object_tracking_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/wang/2.24/object_tracking2/src/object_tracking/msg/DetectedObjectMsg.msg" NAME_WE)
add_custom_target(_object_tracking_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "object_tracking" "/home/wang/2.24/object_tracking2/src/object_tracking/msg/DetectedObjectMsg.msg" "visualization_msgs/Marker:jsk_recognition_msgs/BoundingBox:std_msgs/ColorRGBA:geometry_msgs/Vector3:geometry_msgs/Pose:visualization_msgs/MarkerArray:jsk_recognition_msgs/BoundingBoxArray:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(object_tracking
  "/home/wang/2.24/object_tracking2/src/object_tracking/msg/DetectedObjectMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/visualization_msgs/cmake/../msg/Marker.msg;/opt/ros/melodic/share/jsk_recognition_msgs/cmake/../msg/BoundingBox.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/visualization_msgs/cmake/../msg/MarkerArray.msg;/opt/ros/melodic/share/jsk_recognition_msgs/cmake/../msg/BoundingBoxArray.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/object_tracking
)

### Generating Services

### Generating Module File
_generate_module_cpp(object_tracking
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/object_tracking
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(object_tracking_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(object_tracking_generate_messages object_tracking_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wang/2.24/object_tracking2/src/object_tracking/msg/DetectedObjectMsg.msg" NAME_WE)
add_dependencies(object_tracking_generate_messages_cpp _object_tracking_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(object_tracking_gencpp)
add_dependencies(object_tracking_gencpp object_tracking_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS object_tracking_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(object_tracking
  "/home/wang/2.24/object_tracking2/src/object_tracking/msg/DetectedObjectMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/visualization_msgs/cmake/../msg/Marker.msg;/opt/ros/melodic/share/jsk_recognition_msgs/cmake/../msg/BoundingBox.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/visualization_msgs/cmake/../msg/MarkerArray.msg;/opt/ros/melodic/share/jsk_recognition_msgs/cmake/../msg/BoundingBoxArray.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/object_tracking
)

### Generating Services

### Generating Module File
_generate_module_eus(object_tracking
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/object_tracking
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(object_tracking_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(object_tracking_generate_messages object_tracking_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wang/2.24/object_tracking2/src/object_tracking/msg/DetectedObjectMsg.msg" NAME_WE)
add_dependencies(object_tracking_generate_messages_eus _object_tracking_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(object_tracking_geneus)
add_dependencies(object_tracking_geneus object_tracking_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS object_tracking_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(object_tracking
  "/home/wang/2.24/object_tracking2/src/object_tracking/msg/DetectedObjectMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/visualization_msgs/cmake/../msg/Marker.msg;/opt/ros/melodic/share/jsk_recognition_msgs/cmake/../msg/BoundingBox.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/visualization_msgs/cmake/../msg/MarkerArray.msg;/opt/ros/melodic/share/jsk_recognition_msgs/cmake/../msg/BoundingBoxArray.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/object_tracking
)

### Generating Services

### Generating Module File
_generate_module_lisp(object_tracking
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/object_tracking
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(object_tracking_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(object_tracking_generate_messages object_tracking_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wang/2.24/object_tracking2/src/object_tracking/msg/DetectedObjectMsg.msg" NAME_WE)
add_dependencies(object_tracking_generate_messages_lisp _object_tracking_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(object_tracking_genlisp)
add_dependencies(object_tracking_genlisp object_tracking_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS object_tracking_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(object_tracking
  "/home/wang/2.24/object_tracking2/src/object_tracking/msg/DetectedObjectMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/visualization_msgs/cmake/../msg/Marker.msg;/opt/ros/melodic/share/jsk_recognition_msgs/cmake/../msg/BoundingBox.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/visualization_msgs/cmake/../msg/MarkerArray.msg;/opt/ros/melodic/share/jsk_recognition_msgs/cmake/../msg/BoundingBoxArray.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/object_tracking
)

### Generating Services

### Generating Module File
_generate_module_nodejs(object_tracking
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/object_tracking
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(object_tracking_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(object_tracking_generate_messages object_tracking_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wang/2.24/object_tracking2/src/object_tracking/msg/DetectedObjectMsg.msg" NAME_WE)
add_dependencies(object_tracking_generate_messages_nodejs _object_tracking_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(object_tracking_gennodejs)
add_dependencies(object_tracking_gennodejs object_tracking_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS object_tracking_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(object_tracking
  "/home/wang/2.24/object_tracking2/src/object_tracking/msg/DetectedObjectMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/visualization_msgs/cmake/../msg/Marker.msg;/opt/ros/melodic/share/jsk_recognition_msgs/cmake/../msg/BoundingBox.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/visualization_msgs/cmake/../msg/MarkerArray.msg;/opt/ros/melodic/share/jsk_recognition_msgs/cmake/../msg/BoundingBoxArray.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/object_tracking
)

### Generating Services

### Generating Module File
_generate_module_py(object_tracking
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/object_tracking
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(object_tracking_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(object_tracking_generate_messages object_tracking_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wang/2.24/object_tracking2/src/object_tracking/msg/DetectedObjectMsg.msg" NAME_WE)
add_dependencies(object_tracking_generate_messages_py _object_tracking_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(object_tracking_genpy)
add_dependencies(object_tracking_genpy object_tracking_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS object_tracking_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/object_tracking)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/object_tracking
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(object_tracking_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET visualization_msgs_generate_messages_cpp)
  add_dependencies(object_tracking_generate_messages_cpp visualization_msgs_generate_messages_cpp)
endif()
if(TARGET jsk_recognition_msgs_generate_messages_cpp)
  add_dependencies(object_tracking_generate_messages_cpp jsk_recognition_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/object_tracking)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/object_tracking
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(object_tracking_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET visualization_msgs_generate_messages_eus)
  add_dependencies(object_tracking_generate_messages_eus visualization_msgs_generate_messages_eus)
endif()
if(TARGET jsk_recognition_msgs_generate_messages_eus)
  add_dependencies(object_tracking_generate_messages_eus jsk_recognition_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/object_tracking)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/object_tracking
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(object_tracking_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET visualization_msgs_generate_messages_lisp)
  add_dependencies(object_tracking_generate_messages_lisp visualization_msgs_generate_messages_lisp)
endif()
if(TARGET jsk_recognition_msgs_generate_messages_lisp)
  add_dependencies(object_tracking_generate_messages_lisp jsk_recognition_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/object_tracking)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/object_tracking
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(object_tracking_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET visualization_msgs_generate_messages_nodejs)
  add_dependencies(object_tracking_generate_messages_nodejs visualization_msgs_generate_messages_nodejs)
endif()
if(TARGET jsk_recognition_msgs_generate_messages_nodejs)
  add_dependencies(object_tracking_generate_messages_nodejs jsk_recognition_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/object_tracking)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/object_tracking\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/object_tracking
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(object_tracking_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET visualization_msgs_generate_messages_py)
  add_dependencies(object_tracking_generate_messages_py visualization_msgs_generate_messages_py)
endif()
if(TARGET jsk_recognition_msgs_generate_messages_py)
  add_dependencies(object_tracking_generate_messages_py jsk_recognition_msgs_generate_messages_py)
endif()
