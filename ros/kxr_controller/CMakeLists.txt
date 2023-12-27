cmake_minimum_required(VERSION 2.8.3)
project(kxr_controller)

message(STATUS "CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")

execute_process(
  COMMAND ${CMAKE_CXX_COMPILER} -dumpversion
  OUTPUT_VARIABLE GCC_VERSION
)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm" OR CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  message(STATUS "Configuring for ARM or AArch64 architecture.")
endif()

# To avoid issues with pub/sub in ROS topic on ARM architecture, compile with O2 optimization.
# https://github.com/ros/roscpp_core/pull/136
if(GCC_VERSION VERSION_LESS 9.4.0 AND (CMAKE_SYSTEM_PROCESSOR MATCHES "arm" OR CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64") AND CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_CXX_FLAGS_RELEASE "-O2")
  message(STATUS "GCC version is less than 9.4.0 and target architecture is ARM. Setting CXX_FLAGS_RELEASE to -O2.")
endif()

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
add_dependencies(kxr_controller ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(kxr_controller
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)

install(TARGETS kxr_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
