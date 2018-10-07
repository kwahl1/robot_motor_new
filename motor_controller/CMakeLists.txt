cmake_minimum_required(VERSION 2.8.3)
project(motor_controller)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs phidgets pid)



catkin_package(
  CATKIN_DEPENDS roscpp std_msgs phidgets pid
)

include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
)


add_executable(motor_controller_node src/motor_controller.cpp)
target_link_libraries(motor_controller_node ${catkin_LIBRARIES} )

