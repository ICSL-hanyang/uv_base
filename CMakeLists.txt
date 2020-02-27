cmake_minimum_required(VERSION 2.8.3)
project(uv_base)

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
  nav_msgs
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
   nav_msgs
   std_msgs
   sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES uv_base
  CATKIN_DEPENDS geometry_msgs mavros mavros_extras roscpp std_msgs sensor_msgs
# DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(uv_base_node src/main.cpp)
target_link_libraries(uv_base_node ${catkin_LIBRARIES})
add_executable(map_node src/map.cpp)
target_link_libraries(map_node ${catkin_LIBRARIES})
add_executable(joystick_control_node src/joystick.cpp)
target_link_libraries(joystick_control_node ${catkin_LIBRARIES})
add_executable(sensor_node src/sensor.cpp)
target_link_libraries(sensor_node ${catkin_LIBRARIES})
