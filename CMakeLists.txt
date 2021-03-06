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
  mg_msgs
  tf
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
               src/callbacks.cpp
               src/services.cpp)
target_link_libraries(mediation_layer ${catkin_LIBRARIES})
add_dependencies(mediation_layer ${catkin_EXPORTED_TARGETS})
add_dependencies(mediation_layer mg_msgs_generate_messages_cpp)

install(TARGETS mediation_layer
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(land_quads 
               src/land_quads.cpp)
target_link_libraries(land_quads ${catkin_LIBRARIES})
add_dependencies(land_quads ${catkin_EXPORTED_TARGETS})
add_dependencies(land_quads mg_msgs_generate_messages_cpp)

install(TARGETS land_quads
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})