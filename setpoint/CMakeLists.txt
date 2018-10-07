cmake_minimum_required(VERSION 2.8.3)
project(setpoint)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs std_msgs)



catkin_package(
  INCLUDE_DIRS include
  DEPENDS
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(motor_setpoint_node src/setpoint.cpp)
target_link_libraries(motor_setpoint_node ${catkin_LIBRARIES} ) 

