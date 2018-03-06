cmake_minimum_required(VERSION 2.8.3)
project(mediation_layer)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cmake_modules
  px4_control
)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS Eigen
)


###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(mediation_layer 
               src/mediationLayer.cpp
               src/ml_class.cpp
               src/rk4.cpp
               src/helper.cpp
               src/visualization_functions.cpp
               src/threads.cpp
               src/callbacks.cpp)
target_link_libraries(mediation_layer ${catkin_LIBRARIES})