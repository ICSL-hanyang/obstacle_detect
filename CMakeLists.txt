cmake_minimum_required(VERSION 2.8.3)
project(obstacle_detect)

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
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Pair.msg
  VectorPair.msg
  Orientation.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES obstacle_detect
  CATKIN_DEPENDS roscpp message_runtime std_msgs
  #DEPENDS system_lib
)


include_directories(
  include
  msg
  ${catkin_INCLUDE_DIRS}
)

add_executable(obstacle_detect_node src/obstacle_detect.cpp)
add_dependencies(obstacle_detect_node ${PROJECT_NAME}_generate_message_cpp)

target_link_libraries(obstacle_detect_node ${catkin_LIBRARIES} )

