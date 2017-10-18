SET(PROJECT_NAME djh_ins)

project(${PROJECT_NAME})
cmake_minimum_required(VERSION 2.8.3)

if(NOT ${CMAKE_VERSION} VERSION_LESS 3.0)
    cmake_policy(SET CMP0026 OLD)
endif()

SET(CMAKE_VERBOSE_MAKEFILE ON)

############################################################
# Build type and flags
SET(CMAKE_BUILD_TYPE Release) # Options:  Debug, RelWithDebInfo, Release
SET(CMAKE_CXX_FLAGS "-Wall -Werror -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -O0 -g")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -mmmx -msse -msse -msse2 -msse3 -mssse3 -fomit-frame-pointer")

# Add plain cmake packages
find_package(Eigen3 REQUIRED)

# Add Sources
SET(SOURCES  ${PROJECT_SOURCE_DIR}/src/quaternion_math.cpp
             ${PROJECT_SOURCE_DIR}/src/gravity_model.cpp
             ${PROJECT_SOURCE_DIR}/src/ins_ode.cpp
             ${PROJECT_SOURCE_DIR}/src/preintegration.cpp
             ${PROJECT_SOURCE_DIR}/src/integrator.cpp
             ${PROJECT_SOURCE_DIR}/src/imu_correct.cpp
             ${PROJECT_SOURCE_DIR}/src/djh_ins.cpp
)

# Add library
INCLUDE_DIRECTORIES(include/djh_ins 
                    src
                    ${EIGEN3_INCLUDE_DIRS}
)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})

################################################################################
# Create the djh_insConfig.cmake file for other cmake projects.
GET_TARGET_PROPERTY( FULL_LIBRARY_NAME ${PROJECT_NAME} LOCATION )
SET(djh_ins_LIBRARIES ${FULL_LIBRARY_NAME} )
SET(djh_ins_LIBRARY_DIR ${PROJECT_BINARY_DIR} )
SET(djh_ins_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")
CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/djh_insConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/djh_insConfig.cmake @ONLY IMMEDIATE )
export( PACKAGE djh_ins )

INSTALL(DIRECTORY include/djh_ins DESTINATION ${CMAKE_INSTALL_PREFIX}/include FILES_MATCHING PATTERN "*.h" )
INSTALL(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )

