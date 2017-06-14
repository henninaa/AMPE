#
# Project settings
#

# Minimum required version of cmake
CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

# Project name and programming languages used
PROJECT( MY_COOL_PROJECT CXX )
ADD_DEFINITIONS( -std=c++11 )

# CMake module(s) path
SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )

#
# Prerequisites
#
FIND_PACKAGE( ACADO REQUIRED )

#
# Include directories
#
INCLUDE_DIRECTORIES( . ${ACADO_INCLUDE_DIRS} )

#
# Compiled cpp files
#
file(GLOB SOURCES "source/*.cpp" )

#
# Build an executable
#
ADD_EXECUTABLE(        MPC ${SOURCES} )
# Linux and OS X
TARGET_LINK_LIBRARIES( MPC ${ACADO_SHARED_LIBRARIES} )
# Windows
#TARGET_LINK_LIBRARIES( MPC ${ACADO_STATIC_LIBRARIES} )
SET_TARGET_PROPERTIES( MPC  PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build )
