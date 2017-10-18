cmake_minimum_required(VERSION 2.8.3)
project(djh_ins_test)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  roslib
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules)

# Add plain cmake packages
find_package(Eigen3 REQUIRED)
find_package(djh_ins REQUIRED)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    DEPENDS 
      EIGEN3 
      djh_ins
    CATKIN_DEPENDS 
      roscpp 
      roslib
    INCLUDE_DIRS 
)

# Set link libraries
LIST(APPEND LINK_LIBS 
  ${djh_ins_LIBRARIES}
  ${catkin_LIBRARIES}
)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
      ${EIGEN3_INCLUDE_DIRS}
      ${catkin_INCLUDE_DIRS}
      ${djh_ins_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(djh_ins_node djh_ins_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(djh_ins_node
                      ${LINK_LIBS}
)
