cmake_minimum_required(VERSION 2.8.3)
project(kxr_controller)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  std_msgs
  roscpp
  cmake_modules
  controller_manager
  hardware_interface
  angles
  urdf
)
find_package(Eigen REQUIRED)

add_action_files(
  DIRECTORY action
  FILES ServoOnOff.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp controller_manager hardware_interface angles urdf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(kxr_controller src/kxr_ros_controller.cpp src/kxr_robot_hardware.cpp)
target_link_libraries(kxr_controller
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)

install(TARGETS kxr_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
