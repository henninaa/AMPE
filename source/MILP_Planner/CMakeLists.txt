#
# Project settings
#
project(path_planner CXX)
# Minimum required version of cmake
CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

# Project name and programming languages used
PROJECT( path_planner CXX )
ADD_DEFINITIONS( -std=c++11 )

SET(AMPL_DIRECTORY /home/henning/Documents/prosjektet/ampl/amplapi)

INCLUDE_DIRECTORIES( ${AMPL_DIRECTORY}/include )

file(GLOB SOURCES "source/*.cpp" )

add_executable(pathPlanner ${SOURCES})
target_link_libraries(pathPlanner  ${AMPL_DIRECTORY}/lib/libampl.so)
SET_TARGET_PROPERTIES( pathPlanner  PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build )
