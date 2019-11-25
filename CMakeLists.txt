cmake_minimum_required(VERSION 2.8.3)
project(drone_base)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mavros
  mavros_msgs
  mavros_extras
  geometry_msgs
  geographic_msgs
  sensor_msgs
  message_generation
  tf
)

add_service_files(
   FILES
   srvMultiSetpointLocal.srv
   srvMultiSetpointGlobal.srv
   srvMultiSetVelLocal.srv
   srvGoToVehicle.srv
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  mavros_msgs
  geometry_msgs  
  geographic_msgs
  std_msgs
  tf
)

#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES swarm_ctrl_pkg
#  CATKIN_DEPENDS geometry_msgs mavros mavros_extras roscpp std_msgs
# ## DEPENDS system_lib
#)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES drone_base
  CATKIN_DEPENDS geometry_msgs mavros mavros_extras roscpp std_msgs tf
# DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(drone_base_node src/main.cpp src/vehicle.cpp)
target_link_libraries(drone_base_node ${catkin_LIBRARIES})

add_executable(odometry_tf_pub_node src/odometry_tf_pub.cpp)
target_link_libraries(odometry_tf_pub_node ${catkin_LIBRARIES})

add_executable(depthcamera_link_tf_pub_node src/depthcamera_link_tf_pub.cpp)
target_link_libraries(depthcamera_link_tf_pub_node ${catkin_LIBRARIES})

find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
#target_link_libraries(${OCTOMAP_LIBRARIES})
