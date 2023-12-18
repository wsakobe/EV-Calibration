cmake_minimum_required(VERSION 3.0.2)
project(circle_detector)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(OpenCV 3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  circle_msgs
  cv_bridge
  image_transport
  roscpp
  std_msgs
  dvs_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES circle_detector
#  CATKIN_DEPENDS circle_msgs cv_bridge image_transport opencv3 roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(circle_detector
src/circle_detector.cpp)

target_link_libraries(circle_detector
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_dependencies(circle_detector circle_msgs_generate_messages_cpp)

