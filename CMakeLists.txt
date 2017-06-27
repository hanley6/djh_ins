cmake_minimum_required(VERSION 2.8.3)
project(djh_ins)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  roslib
  image_transport
  cv_bridge
  message_generation
)

# Add plain cmake packages
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
    compute_ins.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    std_msgs  
    # Other packages containing msgs
)

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
      OpenCV
    CATKIN_DEPENDS 
      roscpp 
      image_transport 
      cv_bridge
      message_runtime
    INCLUDE_DIRS 
      include
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(ins_lib
            src/imu_corrector.cpp
            src/aggregator.cpp
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(djh_ins_node src/djh_ins_node.cpp)
# Add additional node
add_executable(bare_bones_node src/bare_bones_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(djh_ins_node
                      ${catkin_LIBRARIES}
                      ins_lib
)

# Now the other node
target_link_libraries(bare_bones_node
                      ${catkin_LIBRARIES}
)

#############
## Install ##
#############


#############
## Testing ##
#############

